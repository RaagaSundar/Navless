#!/usr/bin/env python
"""
step_extract.py: STEP -> tessellated mesh dump, using NATIVE OpenCascade (pip: cadquery-ocp).

Why native and not occt-import-js (WASM)?  The Sky Beta STEP is 78 MB / 447k points; the WASM build
exhausts its 2 GB heap while meshing and silently returns faces with zero triangles. Native OCCT has
no such limit, meshes in parallel, and XDE keeps the assembly names, transforms and colours.

    python -I scripts/step_extract.py --step sky-beta.step --out scratch/skybeta [--defl 0.12] [--ang 0.35]

Writes  <out>/parts.json  (assembly leaves: names, colours, bbox, buffer offsets, per-face colour runs)
        <out>/mesh.bin    (float32 positions [m], float32 normals, uint32 indices; little-endian)

Coordinates are metres, in the STEP's own axes (the Node step-to-glb.mjs step normalises orientation).
"""
import argparse
import json
import os
import sys
import time

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepLib import BRepLib_ToolTriangulatedShape
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence  # OCP >= 8 names it this way
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_ColorGen, XCAFDoc_ColorSurf, XCAFDoc_ColorTool, XCAFDoc_DocumentTool

MM = 0.001  # OCCT reads STEP into millimetres


def log(msg, t0=[time.time()]):
    print(f"[{time.time() - t0[0]:7.1f}s] {msg}", flush=True)


def label_name(label):
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return ""


def get_color(color_tool, target):
    """RGB tuple (0..1) for a label or a shape, or None. Surface colour first, then generic."""
    c = Quantity_Color()
    for kind in (XCAFDoc_ColorSurf, XCAFDoc_ColorGen):
        try:
            if isinstance(target, TDF_Label):
                ok = XCAFDoc_ColorTool.GetColor_s(target, kind, c)
            else:
                ok = color_tool.GetColor(target, kind, c)
        except Exception:
            ok = False
        if ok:
            return (c.Red(), c.Green(), c.Blue())  # linear RGB, which is what glTF wants
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--defl", type=float, default=0.12, help="linear deflection, mm")
    ap.add_argument("--ang", type=float, default=0.35, help="angular deflection, rad")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
    app.InitDocument(doc)

    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)
    reader.SetLayerMode(False)
    reader.SetPropsMode(False)
    reader.SetMatMode(False)
    log(f"reading {args.step} ({os.path.getsize(args.step) / 1048576:.0f} MB)")
    status = reader.ReadFile(args.step)
    if status != IFSelect_RetDone:
        sys.exit(f"ReadFile failed: {status}")
    log("transferring to XDE document")
    if not reader.Transfer(doc):
        sys.exit("Transfer failed")
    log("document ready")

    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    color_tool = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

    leaves = []  # (names, shape(located), definition label, colour)

    def walk(label, loc, names):
        if shape_tool.IsAssembly_s(label):
            comps = TDF_LabelSequence()
            shape_tool.GetComponents_s(label, comps)
            for i in range(1, comps.Length() + 1):
                comp = comps.Value(i)
                ref = TDF_Label()
                if shape_tool.GetReferredShape_s(comp, ref):
                    inst_loc = loc * shape_tool.GetLocation_s(comp)
                    walk(ref, inst_loc, names + [label_name(comp) or label_name(ref)])
        else:
            shp = shape_tool.GetShape_s(label)
            if not shp.IsNull():
                leaves.append((names, shp.Moved(loc), label, get_color(color_tool, label)))

    free = TDF_LabelSequence()
    shape_tool.GetFreeShapes(free)
    for i in range(1, free.Length() + 1):
        lab = free.Value(i)
        walk(lab, TopLoc_Location(), [label_name(lab)])
    log(f"{len(leaves)} assembly leaves")

    parts = []
    chunks = []  # raw bytes
    offset = 0

    def push(arr):
        nonlocal offset
        b = arr.tobytes()
        chunks.append(b)
        info = {"offset": offset, "length": int(arr.size)}
        offset += len(b)
        return info

    total_tris = 0
    for li, (names, shape, label, leaf_color) in enumerate(leaves):
        # mesh this leaf (parallel); relative=False -> absolute deflection in mm
        BRepMesh_IncrementalMesh(shape, args.defl, False, args.ang, True)

        # split by solid so big assemblies (the Jetson devkit etc.) stay addressable
        solids = []
        exp = TopExp_Explorer(shape, TopAbs_SOLID)
        while exp.More():
            solids.append(TopoDS.Solid(exp.Current()))
            exp.Next()
        groups = solids if solids else [shape]  # shells/faces-only leaves fall back to the whole shape

        for si, grp in enumerate(groups):
            pos, nrm, idx, runs = [], [], [], []
            base = 0
            tri_count = 0
            solid_color = None
            if solids:
                sub = TDF_Label()
                try:
                    if shape_tool.FindSubShape(label, grp, sub):
                        solid_color = get_color(color_tool, sub)
                except Exception:
                    solid_color = None
            fexp = TopExp_Explorer(grp, TopAbs_FACE)
            while fexp.More():
                face = TopoDS.Face(fexp.Current())
                fexp.Next()
                loc = TopLoc_Location()
                tri = BRep_Tool.Triangulation_s(face, loc)
                if tri is None:
                    continue
                try:
                    BRepLib_ToolTriangulatedShape.ComputeNormals_s(face, tri)
                except Exception:
                    pass
                trsf = loc.Transformation()
                nn = tri.NbNodes()
                nt = tri.NbTriangles()
                if nn == 0 or nt == 0:
                    continue
                p = np.empty((nn, 3), np.float32)
                for k in range(1, nn + 1):
                    q = tri.Node(k).Transformed(trsf)
                    p[k - 1] = (q.X(), q.Y(), q.Z())
                p *= MM
                has_n = tri.HasNormals()
                n = np.empty((nn, 3), np.float32)
                if has_n:
                    for k in range(1, nn + 1):
                        d = tri.Normal(k).Transformed(trsf)
                        n[k - 1] = (d.X(), d.Y(), d.Z())
                t = np.empty((nt, 3), np.uint32)
                for k in range(1, nt + 1):
                    a, b, c = tri.Triangle(k).Get()
                    t[k - 1] = (a - 1, b - 1, c - 1)
                if face.Orientation() == TopAbs_REVERSED:
                    t = t[:, [0, 2, 1]]
                # geometric check: make sure vertex normals agree with the winding (flip once per face if not)
                e1 = p[t[:, 1]] - p[t[:, 0]]
                e2 = p[t[:, 2]] - p[t[:, 0]]
                gn = np.cross(e1, e2)
                if has_n:
                    vn = n[t[:, 0]]
                    if float(np.sum(np.einsum("ij,ij->i", gn, vn))) < 0:
                        n = -n
                else:
                    # flat-shaded fallback: accumulate face normals into vertices
                    acc = np.zeros_like(p)
                    for col in range(3):
                        np.add.at(acc, t[:, col], gn)
                    ln = np.linalg.norm(acc, axis=1, keepdims=True)
                    n = acc / np.maximum(ln, 1e-12)
                fcol = None
                try:
                    fsub = TDF_Label()
                    if shape_tool.FindSubShape(label, face, fsub):
                        fcol = get_color(color_tool, fsub)
                except Exception:
                    fcol = None
                pos.append(p)
                nrm.append(n.astype(np.float32))
                idx.append(t + base)
                col = fcol or solid_color or leaf_color
                runs.append({"first": tri_count, "last": tri_count + nt - 1, "rgb": list(col) if col else None})
                base += nn
                tri_count += nt
            if tri_count == 0:
                continue
            P = np.concatenate(pos)
            N = np.concatenate(nrm)
            I = np.concatenate(idx).astype(np.uint32)
            bb = [*P.min(axis=0).tolist(), *P.max(axis=0).tolist()]
            parts.append(
                {
                    "id": len(parts),
                    "path": names,
                    "part": names[1] if len(names) > 1 else names[0],
                    "solid": si,
                    "color": list(solid_color or leaf_color) if (solid_color or leaf_color) else None,
                    "bbox": bb,
                    "tris": int(tri_count),
                    "verts": int(P.shape[0]),
                    "pos": push(P.reshape(-1)),
                    "nrm": push(N.reshape(-1)),
                    "idx": push(I.reshape(-1)),
                    "faceColors": runs,
                }
            )
            total_tris += tri_count
        if (li + 1) % 5 == 0 or li + 1 == len(leaves):
            log(f"leaf {li + 1}/{len(leaves)} done, {len(parts)} meshes, {total_tris:,} tris so far")

    with open(os.path.join(args.out, "mesh.bin"), "wb") as fh:
        for b in chunks:
            fh.write(b)
    with open(os.path.join(args.out, "parts.json"), "w", encoding="utf-8") as fh:
        json.dump({"unit": "m", "deflection_mm": args.defl, "angular_rad": args.ang, "parts": parts}, fh)
    log(f"wrote {len(parts)} meshes, {total_tris:,} triangles, {offset / 1048576:.0f} MB -> {args.out}")


if __name__ == "__main__":
    main()
