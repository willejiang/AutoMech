"""workflow4freecad CLI: one product prompt -> an assembled URDF + meshes.

Examples
--------
  py -3.14 main.py --prompt "a simple table: a flat square top on a single central leg"
  py -3.14 main.py --prompt "a 2-DOF pan-tilt camera mount" --show
  py -3.14 main.py --prompt "a desk lamp" --no-viz --allow-partial

Settings resolve as: built-in defaults < $WORKFLOW4FREECAD_CONFIG JSON <
environment variables < these CLI flags. The LLM API key comes from
$FREECAD_AI_API_KEY (the in-code default is a placeholder).
"""

from __future__ import annotations

import argparse
import os
import sys

from config import Settings
from workflow4freecad.orchestrator import Orchestrator, OrchestratorError


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="workflow4freecad",
        description="Decompose a product prompt into parts, build each in "
                    "FreeCAD, and assemble them into a URDF.")
    p.add_argument("--prompt", required=True,
                   help="one-line natural-language product description")
    p.add_argument("--image", default=None,
                   help="optional local image of the product for the manager to "
                        "decompose (.png/.jpg/.jpeg/.gif/.webp); the prompt is "
                        "then treated as a hint")
    p.add_argument("--out", default="output",
                   help="output directory (default: ./output)")
    p.add_argument("--model", default=None,
                   help="override the LLM model id")
    p.add_argument("--max-workers", type=int, default=None,
                   help="max parallel freecadcmd subprocesses (default: 4)")
    p.add_argument("--retries", type=int, default=None,
                   help="rebuild attempts per part (default: 3)")
    p.add_argument("--allow-partial", action="store_true",
                   help="succeed even if some parts fail to build")
    p.add_argument("--no-viz", action="store_true",
                   help="skip the rendered PNG preview and 3D viewer")
    p.add_argument("--show", action="store_true",
                   help="open an interactive 3D viewer of the assembled product")
    p.add_argument("--judge", action="store_true",
                   help="after building, render 6 views and have an evaluator "
                        "agent judge the result; on failure, refine and rebuild "
                        "up to --max-iterations times")
    p.add_argument("--max-iterations", type=int, default=None,
                   help="max generate->judge->refine passes when --judge is set "
                        "(default: 3)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    if args.image is not None and not os.path.isfile(args.image):
        print(f"ERROR: --image file not found: {args.image}", file=sys.stderr)
        return 2

    overrides = {}
    if args.model:
        overrides["model"] = args.model
    if args.max_workers is not None:
        overrides["max_workers"] = args.max_workers
    if args.retries is not None:
        overrides["worker_retries"] = args.retries
    if args.allow_partial:
        overrides["allow_partial"] = True
    if args.no_viz:
        overrides["do_viz"] = False
    settings = Settings.load(**overrides)

    orch = Orchestrator(settings)
    try:
        if args.judge:
            summary = orch.run_with_judge(args.prompt, out_dir=args.out,
                                          image_path=args.image,
                                          max_iterations=args.max_iterations)
        else:
            summary = orch.run(args.prompt, out_dir=args.out,
                               render=not args.no_viz, image_path=args.image)
    except OrchestratorError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 2
    except Exception as e:  # unexpected — surface type for debugging
        print(f"ERROR: {type(e).__name__}: {e}", file=sys.stderr)
        return 2

    if args.show and not args.no_viz and summary.built > 0:
        try:
            from workflow4freecad.viz import show
            print("[viz] opening interactive viewer (close the window to exit)...")
            show(summary.ctx.urdf_path)
        except Exception as e:
            print(f"[viz] viewer unavailable ({type(e).__name__}: {e})")

    return 0 if summary.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
