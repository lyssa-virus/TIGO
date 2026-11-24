import os
import sys


def _sanitize_snap_env() -> None:
    """
    Restaura variáveis GTK/GSettings modificadas pelo VS Code (Snap) antes de
    carregar o Qt5, evitando erro de schema GLib-GIO ao abrir a interface.
    """
    snap_keys = (
        "GSETTINGS_SCHEMA_DIR",
        "GTK_PATH",
        "GTK_EXE_PREFIX",
        "GTK_IM_MODULE_FILE",
        "GIO_MODULE_DIR",
        "XDG_DATA_DIRS",
        "XDG_CONFIG_DIRS",
        "LOCPATH",
    )
    for key in snap_keys:
        orig_key = f"{key}_VSCODE_SNAP_ORIG"
        if orig_key in os.environ:
            orig_val = os.environ[orig_key]
            if orig_val:
                os.environ[key] = orig_val
            else:
                os.environ.pop(key, None)

    for key in ("GSETTINGS_SCHEMA_DIR", "GTK_PATH", "GTK_EXE_PREFIX", "GTK_IM_MODULE_FILE"):
        if "/snap/" in os.environ.get(key, ""):
            os.environ.pop(key, None)


_sanitize_snap_env()

# Garante execução limpa caso o módulo seja chamado de outro diretório
APP_DIR = os.path.dirname(os.path.abspath(__file__))
if APP_DIR not in sys.path:
    sys.path.insert(0, APP_DIR)

from PyQt5.QtWidgets import QApplication
from main_window import MainWindow


def main() -> int:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setApplicationName("TIGO Telemetry")
    window = MainWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
