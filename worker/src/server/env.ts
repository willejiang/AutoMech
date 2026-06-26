export function env(name: string): string {
  return process.env[name] ?? '';
}

export function requiredEnv(name: string): string {
  const value = env(name);
  if (!value) throw new Error(`${name} is not set`);
  return value;
}

// Base URL of the local LLM gateway that proxies Claude/Gemini/GPT without
// requiring real API keys. All text/LLM calls route through this instead of
// hitting each provider directly.
export function gatewayBaseUrl(): string {
  return env('LLM_GATEWAY_URL').replace(/\/+$/, '') || 'http://localhost:8313';
}

export function webhookBaseUrl(requestUrl: string): string {
  const configuredUrl = env('WEBHOOK_BASE_URL');
  if (configuredUrl) return configuredUrl.replace(/\/$/, '');
  return new URL(requestUrl).origin;
}
