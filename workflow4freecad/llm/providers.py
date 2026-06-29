"""LLM provider registry.

Each provider defines its base URL, default model, API style
(either 'anthropic' for Anthropic's native API, or 'openai' for
OpenAI-compatible /chat/completions endpoints), and whether it
supports native tool calling.

Vendored from freecad-ai with one added entry, ``local_gateway``, for the
locally-deployed OpenAI-compatible gateway this project drives.
"""

PROVIDERS = {
    # Locally-deployed OpenAI-compatible gateway (ghc-api). The /v1 suffix is
    # required: the client appends /chat/completions to base_url. Tool calling
    # is left unverified here, so the manager/worker use plain JSON, not
    # native tool calls.
    "local_gateway": {
        "base_url": "http://127.0.0.1:8313/v1",
        "default_model": "claude-opus-4.8",
        "api_style": "openai",
        "supports_tools": False,
    },
    "anthropic": {
        "base_url": "https://api.anthropic.com",
        "default_model": "claude-sonnet-4-6",
        "api_style": "anthropic",
        "supports_tools": True,
    },
    "openai": {
        "base_url": "https://api.openai.com/v1",
        "default_model": "gpt-4o",
        "api_style": "openai",
        "supports_tools": True,
    },
    "ollama": {
        "base_url": "http://localhost:11434/v1",
        "default_model": "llama3",
        "api_style": "openai",
        "supports_tools": True,
    },
    "openrouter": {
        "base_url": "https://openrouter.ai/api/v1",
        "default_model": "anthropic/claude-sonnet-4-6",
        "api_style": "openai",
        "supports_tools": True,
    },
    "custom": {
        "base_url": "",
        "default_model": "",
        "api_style": "openai",
        "supports_tools": False,
    },
}


def get_api_style(provider_name: str) -> str:
    """Return the API style for a provider ('anthropic' or 'openai')."""
    return PROVIDERS.get(provider_name, {}).get("api_style", "openai")


def supports_tools(provider_name: str) -> bool:
    """Return whether a provider supports native tool calling."""
    return PROVIDERS.get(provider_name, {}).get("supports_tools", False)


def get_default_params(provider_name: str) -> dict:
    """Return the default model parameters for a provider (empty if none)."""
    return dict(PROVIDERS.get(provider_name, {}).get("default_params", {}))


def get_provider_names() -> list[str]:
    """Return list of available provider names."""
    return list(PROVIDERS.keys())
