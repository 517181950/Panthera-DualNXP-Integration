"""Small GLFW probe used to reject accidental CPU OpenGL rendering."""

from __future__ import annotations

import glfw
from OpenGL import GL


SOFTWARE_RENDERER_MARKERS = ("llvmpipe", "softpipe", "swrast", "software rasterizer")


def detect_opengl_renderer() -> tuple[str, str, str]:
    if not glfw.init():
        raise RuntimeError(
            "GLFW could not connect to the desktop. Run from a Wayland/X11 desktop terminal."
        )

    window = None
    try:
        glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
        window = glfw.create_window(64, 64, "Dishwasher GPU probe", None, None)
        if window is None:
            raise RuntimeError("GLFW could not create an OpenGL context")
        glfw.make_context_current(window)
        vendor = GL.glGetString(GL.GL_VENDOR).decode("utf-8", errors="replace")
        renderer = GL.glGetString(GL.GL_RENDERER).decode("utf-8", errors="replace")
        version = GL.glGetString(GL.GL_VERSION).decode("utf-8", errors="replace")
        return vendor, renderer, version
    finally:
        if window is not None:
            glfw.destroy_window(window)
        glfw.terminate()


def require_hardware_opengl(*, allow_software: bool = False) -> str:
    vendor, renderer, version = detect_opengl_renderer()
    print(f"OpenGL vendor:   {vendor}")
    print(f"OpenGL renderer: {renderer}")
    print(f"OpenGL version:  {version}")

    is_software = any(marker in renderer.lower() for marker in SOFTWARE_RENDERER_MARKERS)
    if is_software and not allow_software:
        raise RuntimeError(
            "Software OpenGL renderer detected. Expose /dev/dri, install the Intel/Mesa "
            "driver, and make sure the current user can access the render device. "
            "Use --allow-software-rendering only for diagnostics."
        )
    return renderer
