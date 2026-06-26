import OpenAI from 'openai';
import { gatewayBaseUrl, requiredEnv } from './env';

export type ModelEvaluation = {
  passed: boolean;
  reason: string;
  suggestions: string;
};

// Sentinel the model emits between its prose review and the final JSON verdict.
// The client streams everything BEFORE this marker as readable text, then parses
// the JSON after it for the pass/fail verdict.
export const VERDICT_MARKER = '===VERDICT===';

// Sentinel the SERVER injects once, right before the first real review token, to
// separate Claude's streamed thinking (shown as a live preview so the panel
// never sits on dead air) from the actual review. `reasoning_effort: 'low'`
// usually suppresses thinking but the gateway honors it inconsistently, so we
// still handle the thinking case. Everything before this marker is thinking;
// everything after is the review (which itself ends with VERDICT_MARKER).
export const REVIEW_MARKER = '===REVIEW===';

const EVAL_SYSTEM_PROMPT = `You are a strict 3D CAD model reviewer. The user gives you their original modeling request plus 6 orthographic renders of the generated model from different viewpoints. The views are not labeled — infer each view's orientation yourself from the images.

Analyze step by step, in English, whether the generated model reasonably satisfies the request: are the main features present, are the shape and proportions correct, is anything missing / intersecting / floating / non-printable / over-simplified, does it match the description overall. If it is not acceptable, state clearly what is missing or wrong and give concrete improvement suggestions.

Output requirements (follow strictly):
1. First output the readable review text in English (this part is shown to the user live).
2. After the full analysis, on a new line output the following marker exactly once, immediately followed by a single line of strict JSON:
${VERDICT_MARKER}
{"passed": true or false, "reason": "<short English explanation of the verdict>", "suggestions": "<English improvement suggestions; empty string if passed>"}

"passed" is true only when the model acceptably satisfies the request (main features present, shape/proportions correct, no severe defects); otherwise false. After the marker put ONLY the JSON, nothing else.`;

let client: OpenAI | undefined;
function getEvalClient(): OpenAI {
  // Route through the local gateway's OpenAI-compatible endpoint (no real key
  // needed — the gateway injects credentials). Separate from mesh.ts's
  // getOpenAI(), which talks to the real OpenAI for image generation.
  client ??= new OpenAI({
    apiKey: requiredEnv('OPENAI_API_KEY'),
    baseURL: `${gatewayBaseUrl()}/v1`,
  });
  return client;
}

/**
 * Stream a Claude Opus 4.8 (vision, via the gateway's OpenAI-compatible
 * endpoint) review of whether the generated model satisfies the user's request,
 * given the prompt and the six orthographic-view PNGs. Returns a plain-text
 * ReadableStream: optional thinking preview, then a `${REVIEW_MARKER}`, then the
 * readable review prose, then a `${VERDICT_MARKER}` line followed by the JSON
 * verdict. Runs with low reasoning effort to keep latency down; thinking is
 * forwarded as a preview for the cases where Claude still thinks. On any setup
 * failure the stream closes empty so the caller never breaks.
 */
export function evaluateModelStream({
  prompt,
  images,
}: {
  prompt: string;
  images: string[];
}): ReadableStream<Uint8Array> {
  const encoder = new TextEncoder();

  return new ReadableStream<Uint8Array>({
    async start(controller) {
      if (images.length === 0) {
        controller.close();
        return;
      }
      try {
        const completion = await getEvalClient().chat.completions.create({
          model: 'claude-opus-4.8',
          stream: true,
          // 'low' keeps Claude from spending ~half the wall-clock thinking
          // before any visible token. With it, content starts in ~5s instead of
          // ~9s and the whole eval finishes ~40% faster, with no quality loss
          // worth caring about for a 6-view sanity check.
          reasoning_effort: 'low',
          messages: [
            { role: 'system', content: EVAL_SYSTEM_PROMPT },
            {
              role: 'user',
              content: [
                {
                  type: 'text',
                  text: `The user's original modeling request:\n${prompt || '(no text description provided)'}\n\nBelow are 6 orthographic renders of the generated model. Evaluate based on them.`,
                },
                ...images.map(
                  (url) =>
                    ({
                      type: 'image_url',
                      image_url: { url },
                    }) as const,
                ),
              ],
            },
          ],
        });

        let reviewStarted = false;
        for await (const chunk of completion) {
          const delta = chunk.choices[0]?.delta as
            | { content?: string | null; reasoning_text?: string | null }
            | undefined;
          if (!delta) continue;

          // Forward Claude's thinking as a live preview so the panel shows
          // activity immediately instead of dead air during the (sometimes
          // ~9s) reasoning phase. Once real review content begins, emit
          // REVIEW_MARKER so the client drops the thinking preview and keeps
          // only the actual review.
          if (!reviewStarted && delta.reasoning_text) {
            controller.enqueue(encoder.encode(delta.reasoning_text));
          }
          if (delta.content) {
            if (!reviewStarted) {
              reviewStarted = true;
              controller.enqueue(encoder.encode(REVIEW_MARKER));
            }
            controller.enqueue(encoder.encode(delta.content));
          }
        }
        controller.close();
      } catch (error) {
        // Best-effort: close the stream so the client falls back to "no verdict"
        // instead of erroring. Logged server-side for debugging.
        console.error('[evaluate-model] stream failed:', error);
        controller.close();
      }
    },
  });
}
