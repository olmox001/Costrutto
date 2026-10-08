# SDL3 verification (headers from SDL-main, version 3.5.0)

Source headers extracted from user-provided archive (local ZIP records):
`SDL.h`, `SDL_events.h`, `SDL_render.h`.

## API used by `nqg_sdl_platform.hpp` — status

| Call / symbol | SDL3 3.5.0 | Notes |
|---------------|------------|-------|
| `SDL_EVENT_KEY_DOWN/UP` | OK | event type enum |
| `SDL_EVENT_MOUSE_*` | OK | |
| `SDL_EVENT_FINGER_DOWN/MOTION/UP` | OK | touch → virtual sticks |
| `SDL_CreateTexture` + `RGBA32` + `STREAMING` | OK | framebuffer upload |
| `SDL_UpdateTexture` | OK | |
| `SDL_RenderTexture` | OK | not SDL2 `Copy` |
| `SDL_OpenAudioDeviceStream` | OK | SDL3 audio stream |
| `SDL_PutAudioStreamData` | OK | |

## Transport matrix

| Platform | Input path | Testable here |
|----------|------------|---------------|
| CLI | Commands text | yes |
| SDL desktop | SdlPlatform events | needs libSDL3 |
| iOS + SDL | finger events already mapped | needs Xcode |
| iOS no SDL | CLI / Host only | yes headless |

## Build

`NQG_WITH_SDL` when `SDL_FOUND=1` in `mk/platform.mk`. Without SDL, cleanroom/host/cli still run.
