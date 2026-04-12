#!/usr/bin/env python3
"""
graphify - Code visualisation and dependency graph tool for NRFGate.

Usage:
  graphify install [--platform PLATFORM]
  graphify generate [--format FORMAT] [--output FILE]

PLATFORM: windows | linux | macos   (default: auto-detect)
FORMAT:   mermaid | dot | svg | png (default: mermaid)
"""

import argparse
import os
import platform
import re
import subprocess
import sys
from pathlib import Path

# Absolute path to the repository root (one level above tools/)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = PROJECT_ROOT / "src"

# C keywords and common macros that should not appear as graph nodes
_SKIP = frozenset(
    {
        "if", "for", "while", "do", "switch", "return", "sizeof", "typeof",
        "offsetof", "NULL", "true", "false",
        # Zephyr log macros
        "LOG_DBG", "LOG_INF", "LOG_WRN", "LOG_ERR", "LOG_HEXDUMP_DBG",
        # Zephyr helpers
        "BUILD_ASSERT", "ARRAY_SIZE", "K_MSEC", "K_SECONDS", "K_NO_WAIT",
        "BIT", "IS_ENABLED", "CONTAINER_OF",
    }
)


# ---------------------------------------------------------------------------
# Platform helpers
# ---------------------------------------------------------------------------

def _current_platform() -> str:
    """Return 'windows', 'linux', or 'macos'."""
    s = platform.system().lower()
    if s == "windows":
        return "windows"
    if s == "darwin":
        return "macos"
    return "linux"


def _run(cmd: list, *, check: bool = False) -> tuple[int, str, str]:
    """Run a subprocess and return (returncode, stdout, stderr)."""
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, check=check)
        return r.returncode, r.stdout.strip(), r.stderr.strip()
    except FileNotFoundError:
        return 127, "", f"command not found: {cmd[0]}"
    except subprocess.CalledProcessError as e:
        return e.returncode, e.stdout.strip(), e.stderr.strip()


def _has(cmd: str) -> bool:
    """Return True if *cmd* is available on PATH."""
    probe = "where" if platform.system() == "Windows" else "which"
    code, _, _ = _run([probe, cmd])
    return code == 0


def _pip_install(*packages: str) -> bool:
    """Install one or more pip packages; return True on success."""
    code, _, err = _run([sys.executable, "-m", "pip", "install", "--quiet", *packages])
    if code != 0:
        print(f"    pip install failed: {err}")
    return code == 0


# ---------------------------------------------------------------------------
# install subcommand
# ---------------------------------------------------------------------------

def _install_graphviz_windows() -> bool:
    """
    Try to install the Graphviz native binary on Windows via one of the
    common package managers (winget → chocolatey → scoop).
    Returns True if a manager succeeded.
    """
    managers = [
        (["winget", "install", "--id", "Graphviz.Graphviz", "-e", "--silent"],  "winget"),
        (["choco",  "install", "graphviz", "-y"],                                "chocolatey"),
        (["scoop",  "install", "graphviz"],                                      "scoop"),
    ]
    for cmd, name in managers:
        if not _has(cmd[0]):
            continue
        print(f"    Installing Graphviz via {name} …")
        code, _, err = _run(cmd)
        if code == 0:
            print(f"    Graphviz installed via {name}.")
            return True
        print(f"    {name} failed: {err}")

    print("    No supported package manager found (winget / choco / scoop).")
    print("    Please install Graphviz manually: https://graphviz.org/download/")
    return False


def install_windows() -> bool:
    """Install all Windows dependencies required by graphify."""
    print("[graphify] Installing Windows dependencies …\n")

    ok = True

    # ── Python check ────────────────────────────────────────────────────────
    print("  [1/4] Python")
    _, ver, _ = _run([sys.executable, "--version"])
    print(f"        {ver} — OK")

    # ── pip packages ────────────────────────────────────────────────────────
    print("  [2/4] Python packages (graphviz, pycparser, networkx, matplotlib)")
    req = Path(__file__).parent / "requirements.txt"
    if req.exists():
        code, _, err = _run(
            [sys.executable, "-m", "pip", "install", "--quiet", "-r", str(req)]
        )
        if code == 0:
            print("        Installed from requirements.txt — OK")
        else:
            print(f"        Warning: {err}")
            ok = False
    else:
        if _pip_install("graphviz", "pycparser", "networkx", "matplotlib"):
            print("        Installed inline packages — OK")
        else:
            ok = False

    # ── Graphviz binary ─────────────────────────────────────────────────────
    print("  [3/4] Graphviz binary (dot)")
    if _has("dot"):
        _, ver, _ = _run(["dot", "-V"])
        print(f"        {ver} — OK")
    else:
        print("        Not found — attempting auto-install …")
        if not _install_graphviz_windows():
            ok = False

    # ── west (Zephyr build tool) ────────────────────────────────────────────
    print("  [4/4] west (Zephyr meta-tool)")
    if _has("west"):
        _, ver, _ = _run(["west", "--version"])
        print(f"        {ver} — OK")
    else:
        print("        Not found — installing via pip …")
        if _pip_install("west"):
            print("        west installed — OK")
        else:
            print("        Warning: could not install west automatically.")
            ok = False

    print()
    if ok:
        print("[graphify] Windows installation complete.")
        print("           Restart your terminal if PATH changes were made.")
    else:
        print("[graphify] Installation finished with warnings (see above).")
    return ok


def install_linux() -> bool:
    """Install Linux dependencies."""
    print("[graphify] Installing Linux dependencies …\n")

    pkg_cmds = {
        "apt-get": ["sudo", "apt-get", "install", "-y", "graphviz", "python3-pip"],
        "dnf":     ["sudo", "dnf",     "install", "-y", "graphviz", "python3-pip"],
        "pacman":  ["sudo", "pacman",  "-S", "--noconfirm", "graphviz", "python-pip"],
        "zypper":  ["sudo", "zypper",  "install", "-y", "graphviz", "python3-pip"],
    }

    installed = False
    for mgr, cmd in pkg_cmds.items():
        if _has(mgr):
            if mgr == "apt-get":
                _run(["sudo", "apt-get", "update", "-qq"])
            code, _, err = _run(cmd)
            if code == 0:
                installed = True
            else:
                print(f"  Warning ({mgr}): {err}")
            break

    if not installed and not _has("dot"):
        print("  Could not install Graphviz via system package manager.")
        print("  Please install graphviz manually and re-run.")

    req = Path(__file__).parent / "requirements.txt"
    if req.exists():
        _run([sys.executable, "-m", "pip", "install", "--quiet", "-r", str(req)])
    else:
        _pip_install("graphviz", "pycparser", "networkx", "matplotlib")

    if not _has("west"):
        _pip_install("west")

    print("[graphify] Linux installation complete.")
    return True


def install_macos() -> bool:
    """Install macOS dependencies."""
    print("[graphify] Installing macOS dependencies …\n")

    if _has("brew"):
        _run(["brew", "install", "graphviz"])
    else:
        print("  Homebrew not found; please install from https://brew.sh")
        print("  then re-run: graphify install --platform macos")

    req = Path(__file__).parent / "requirements.txt"
    if req.exists():
        _run([sys.executable, "-m", "pip", "install", "--quiet", "-r", str(req)])
    else:
        _pip_install("graphviz", "pycparser", "networkx", "matplotlib")

    if not _has("west"):
        _pip_install("west")

    print("[graphify] macOS installation complete.")
    return True


def cmd_install(args) -> None:
    target = (args.platform or _current_platform()).lower()
    dispatch = {
        "windows": install_windows,
        "linux":   install_linux,
        "macos":   install_macos,
    }
    fn = dispatch.get(target)
    if fn is None:
        print(f"[graphify] Unknown platform '{target}'. Choose: windows | linux | macos")
        sys.exit(1)
    if not fn():
        sys.exit(1)


# ---------------------------------------------------------------------------
# generate subcommand
# ---------------------------------------------------------------------------

def _parse_calls(path: Path) -> dict[str, set[str]]:
    """
    Lightweight C call-graph extractor.
    Returns {caller: {callee, …}, …} for functions defined in *path*.
    """
    graph: dict[str, set[str]] = {}
    try:
        src = path.read_text(errors="replace")
    except OSError:
        return graph

    # Detect top-level function definitions (non-indented, ends with '{')
    func_def = re.compile(r"^[\w\s\*]+\b(\w+)\s*\([^)]*\)\s*\{", re.MULTILINE)
    call_pat = re.compile(r"\b(\w+)\s*\(")

    current: str | None = None
    depth = 0

    for line in src.splitlines():
        stripped = line.strip()

        if depth == 0:
            m = func_def.match(stripped)
            if m:
                current = m.group(1)
                graph.setdefault(current, set())

        if current:
            for cm in call_pat.finditer(stripped):
                name = cm.group(1)
                if name not in _SKIP and name != current:
                    graph[current].add(name)

        depth += stripped.count("{") - stripped.count("}")
        if depth <= 0:
            depth = 0
            if current and "}" in stripped:
                current = None

    return graph


def _merge_graphs(graphs: list[dict]) -> dict[str, set[str]]:
    merged: dict[str, set[str]] = {}
    for g in graphs:
        for fn, callees in g.items():
            merged.setdefault(fn, set()).update(callees)
    return merged


def _to_mermaid(graph: dict[str, set[str]], max_edges: int = 200) -> str:
    known = set(graph)
    lines = ["flowchart TD"]
    count = 0
    for caller in sorted(graph):
        for callee in sorted(graph[caller]):
            if callee in known and count < max_edges:
                lines.append(f"    {caller} --> {callee}")
                count += 1
    if count == max_edges:
        lines.append(f"    %% (truncated at {max_edges} edges)")
    return "\n".join(lines)


def _to_dot(graph: dict[str, set[str]]) -> str:
    known = set(graph)
    lines = [
        "digraph NRFGate {",
        '    node [shape=box fontname="Courier New"];',
        '    graph [rankdir=LR];',
    ]
    for caller in sorted(graph):
        for callee in sorted(graph[caller]):
            if callee in known:
                lines.append(f'    "{caller}" -> "{callee}";')
    lines.append("}")
    return "\n".join(lines)


def cmd_generate(args) -> None:
    src_files = sorted(SRC_DIR.glob("*.c")) if SRC_DIR.exists() else []
    if not src_files:
        print(f"[graphify] No .c files found in {SRC_DIR}")
        sys.exit(1)

    print(f"[graphify] Parsing {len(src_files)} source file(s) …")
    graph = _merge_graphs([_parse_calls(f) for f in src_files])
    print(f"           {len(graph)} functions, "
          f"{sum(len(v) for v in graph.values())} call edges found.")

    fmt = args.format
    out_base = Path(args.output)

    if fmt == "mermaid":
        content = _to_mermaid(graph)
        out = out_base.with_suffix(".md")
        out.write_text(f"```mermaid\n{content}\n```\n")
        print(f"[graphify] Mermaid diagram  → {out}")

    elif fmt == "dot":
        content = _to_dot(graph)
        out = out_base.with_suffix(".dot")
        out.write_text(content)
        print(f"[graphify] Graphviz DOT     → {out}")

    elif fmt in ("svg", "png"):
        if not _has("dot"):
            print("[graphify] 'dot' not found. Run: graphify install")
            sys.exit(1)
        dot_path = out_base.with_suffix(".dot")
        dot_path.write_text(_to_dot(graph))
        out = out_base.with_suffix(f".{fmt}")
        code, _, err = _run(["dot", f"-T{fmt}", str(dot_path), "-o", str(out)])
        dot_path.unlink(missing_ok=True)
        if code == 0:
            print(f"[graphify] {fmt.upper()} image          → {out}")
        else:
            print(f"[graphify] Graphviz error: {err}")
            sys.exit(1)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        prog="graphify",
        description="Code visualisation and dependency graph tool for NRFGate.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  graphify install --platform windows\n"
            "  graphify install\n"
            "  graphify generate --format mermaid --output callgraph\n"
            "  graphify generate --format png     --output callgraph\n"
        ),
    )
    sub = parser.add_subparsers(dest="command", metavar="COMMAND")
    sub.required = True

    # install
    p_install = sub.add_parser("install", help="Install platform-specific dependencies.")
    p_install.add_argument(
        "--platform",
        choices=["windows", "linux", "macos"],
        default=None,
        help="Target platform (default: auto-detect current OS).",
    )

    # generate
    p_gen = sub.add_parser("generate", help="Generate a call graph from C sources.")
    p_gen.add_argument(
        "--format", "-f",
        choices=["mermaid", "dot", "svg", "png"],
        default="mermaid",
        help="Output format (default: mermaid).",
    )
    p_gen.add_argument(
        "--output", "-o",
        default="callgraph",
        help="Output file base name without extension (default: callgraph).",
    )

    args = parser.parse_args()
    if args.command == "install":
        cmd_install(args)
    elif args.command == "generate":
        cmd_generate(args)


if __name__ == "__main__":
    main()
