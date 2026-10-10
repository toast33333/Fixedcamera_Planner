#!/usr/bin/env python3
"""Add a large non-colliding color marker to a generated Iris SDF."""

import argparse
import xml.etree.ElementTree as ET


def add_text(parent, name, text):
    child = ET.SubElement(parent, name)
    child.text = text
    return child


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("sdf_path")
    parser.add_argument("--rgba", nargs=4, type=float, required=True)
    parser.add_argument("--radius", type=float, default=1.25)
    parser.add_argument("--height", type=float, default=2.3)
    args = parser.parse_args()

    tree = ET.parse(args.sdf_path)
    root = tree.getroot()
    base_link = None
    for link in root.iter("link"):
        if link.get("name") == "base_link":
            base_link = link
            break
    if base_link is None:
        raise SystemExit("base_link not found in " + args.sdf_path)

    visual = ET.SubElement(base_link, "visual", {"name": "client_demo_marker"})
    add_text(visual, "pose", "0 0 %.6g 0 0 0" % args.height)
    geometry = ET.SubElement(visual, "geometry")
    sphere = ET.SubElement(geometry, "sphere")
    add_text(sphere, "radius", "%.6g" % args.radius)
    material = ET.SubElement(visual, "material")
    color = " ".join("%.6g" % value for value in args.rgba)
    add_text(material, "ambient", color)
    add_text(material, "diffuse", color)
    add_text(material, "emissive", color)
    add_text(visual, "cast_shadows", "false")
    add_text(visual, "transparency", "0.12")
    tree.write(args.sdf_path, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    main()
