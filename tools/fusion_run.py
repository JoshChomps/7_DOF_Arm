"""Runs a tools/fusion_arm.py command inside Fusion through the MCP add-in (raw HTTP, port in fusion_sweep.URL).

    software/.venv/Scripts/python.exe tools/fusion_run.py build C34 housing [--save]
    software/.venv/Scripts/python.exe tools/fusion_run.py ids

Sends a few lines that import fusion_lib and fusion_arm from this folder (reloaded each time, since Fusion keeps
modules between runs), so the builders can be any size: the add-in drops scripts over ~16 KB.
"""
import json
import sys
from pathlib import Path

from fusion_sweep import post

STUB = '''import sys, importlib
sys.path.insert(0, {tools!r})
import fusion_lib, fusion_arm
importlib.reload(fusion_lib)
importlib.reload(fusion_arm)


def run(_context: str):
    try:
        out = str(fusion_arm.cli({args!r}))
    except Exception:
        import traceback
        out = "ERROR " + traceback.format_exc()
    open({out!r}, "w", encoding="utf-8").write(out)     # the add-in's print capture can stop working
    print(out)
'''

if __name__ == "__main__":
    out = Path(__file__).parent / ".fusion_out.txt"
    out.unlink(missing_ok=True)
    src = STUB.format(tools=str(Path(__file__).parent), args=sys.argv[1:], out=str(out))
    sid, _ = post({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {
        "protocolVersion": "2025-03-26", "capabilities": {}, "clientInfo": {"name": "deskarm", "version": "1"}}})
    post({"jsonrpc": "2.0", "method": "notifications/initialized"}, sid)
    _, res = post({"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": {
        "name": "fusion_mcp_execute", "arguments": {"featureType": "script", "object": {"script": src}}}}, sid)
    reply = json.loads(res["result"]["content"][0]["text"])
    text = out.read_text(encoding="utf-8") if out.exists() else (reply.get("message") or reply.get("error"))
    print(text)
    sys.exit(0 if reply.get("success") and not str(text).startswith("ERROR") else 1)
