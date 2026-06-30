"""Shared helper: read a local image into a provider-neutral conversation block.

The manager attaches one product image; the judger attaches up to six rendered
views (plus an optional reference photo). Both need the same
``{"type":"image","media_type":...,"data":<base64>}`` block that
``Conversation.add_user_message(images=...)`` expects, so the loader lives here
instead of inside the manager.
"""

from __future__ import annotations

import base64
import os


class ImageLoadError(RuntimeError):
    """An image could not be turned into a conversation block (type/read/empty)."""


# The image types the gateway's vision model accepts, keyed by file extension.
_IMAGE_MEDIA_TYPES = {
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".gif": "image/gif",
    ".webp": "image/webp",
}


def load_image_block(path: str) -> dict:
    """Read a local image into the conversation's provider-neutral image block.

    Returns ``{"type":"image","media_type":...,"data":<base64>}`` -- the shape
    Conversation.add_user_message(images=...) expects, which it then converts to
    the OpenAI ``image_url`` data-URI. Raises ImageLoadError with a clear message
    on an unsupported type, an unreadable file, or an empty file.
    """
    ext = os.path.splitext(path)[1].lower()
    media_type = _IMAGE_MEDIA_TYPES.get(ext)
    if media_type is None:
        raise ImageLoadError(
            f"Unsupported image type '{ext}' for {path}. Supported: "
            f"{', '.join(sorted(_IMAGE_MEDIA_TYPES))}")
    try:
        with open(path, "rb") as f:
            data = f.read()
    except OSError as e:
        raise ImageLoadError(f"Could not read image '{path}': {e}") from e
    if not data:
        raise ImageLoadError(f"Image file is empty: {path}")
    return {
        "type": "image",
        "media_type": media_type,
        "data": base64.b64encode(data).decode("ascii"),
    }
