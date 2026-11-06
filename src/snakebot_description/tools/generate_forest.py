#!/usr/bin/python3
"""Generate the world and its matching collision map from one fixed seed."""

import argparse
import math
from pathlib import Path
import random
import xml.etree.ElementTree as E


def element(parent, tag, text=None, **attributes):
    node = E.SubElement(parent, tag, attributes)
    if text is not None:
        node.text = str(text)
    return node


def shape(link, kind, name, geometry, pose='0 0 0 0 0 0', color=None):
    node = element(link, kind, name=name)
    element(node, 'pose', pose)
    g = element(element(node, 'geometry'), geometry[0])
    for key, value in geometry[1].items():
        element(g, key, value)
    if color:
        material = element(node, 'material')
        element(material, 'ambient', color)
        element(material, 'diffuse', color)
        element(material, 'specular', '0.035 0.03 0.025 1')
        metal = element(element(material, 'pbr'), 'metal')
        element(metal, 'metalness', 0)
        element(metal, 'roughness', 0.95)
    return node


def model(world, name, pose='0 0 0 0 0 0'):
    node = element(world, 'model', name=name)
    element(node, 'static', 'true')
    element(node, 'pose', pose)
    return element(node, 'link', name='geometry')


def generate(seed=73, count=32):
    if count < 25:
        raise ValueError('At least 25 trees are required')
    rng = random.Random(seed)
    root = E.Element('sdf', version='1.12')
    world = element(root, 'world', name='snake_world')
    physics = element(world, 'physics', name='1ms', type='ignored')
    element(physics, 'max_step_size', 0.001)
    element(physics, 'real_time_factor', 1.0)
    element(world, 'gravity', '0 0 -9.81')
    for filename, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                           ('scene-broadcaster', 'SceneBroadcaster')]:
        element(world, 'plugin', filename=f'gz-sim-{filename}-system', name=f'gz::sim::systems::{name}')
    scene = element(world, 'scene')
    element(scene, 'grid', 'false')
    element(scene, 'ambient', '0.58 0.55 0.49 1')
    element(scene, 'background', '0.66 0.70 0.72 1')
    element(scene, 'shadows', 'true')
    light = element(world, 'light', name='morning_sun', type='directional')
    element(light, 'pose', '-10 -8 12 0 0 0')
    element(light, 'direction', '0.82 0.35 -0.26')
    element(light, 'diffuse', '0.95 0.84 0.66 1')
    element(light, 'specular', '0.9 0.84 0.72 1')
    element(light, 'cast_shadows', 'true')
    # Surface z=0.025 also physically covers a GUI grid at z=0.
    ground_z = 0.025
    ground = model(world, 'solid_soil', '0 0 -0.025 0 0 0')
    box = ('box', {'size': '50 50 0.1'})
    shape(ground, 'collision', 'soil_collision', box)
    shape(ground, 'visual', 'soil_visual', box, color='0.19 0.125 0.068 1')
    trees = []
    while len(trees) < count:
        x, y = rng.uniform(-14.5, 14.5), rng.uniform(-14.5, 14.5)
        if math.hypot(x, y) < 3.6 or any(math.hypot(x-t[0], y-t[1]) < 2.0 for t in trees):
            continue
        trees.append((x, y, rng.uniform(0.15, 0.3), rng.uniform(2.0, 4.0)))
    palettes = [(0.12, 0.24, 0.08), (0.36, 0.43, 0.095), (0.65, 0.25, 0.07)]
    for index, (x, y, radius, height) in enumerate(trees):
        link = model(world, f'tree_{index:02}', f'{x:.5f} {y:.5f} {ground_z} 0 0 0')
        trunk = ('cylinder', {'radius': f'{radius:.5f}', 'length': f'{height:.5f}'})
        trunk_pose = f'0 0 {height/2:.5f} 0 0 0'
        shape(link, 'collision', 'trunk_collision', trunk, trunk_pose)
        shape(link, 'visual', 'trunk_visual', trunk, trunk_pose, '0.14 0.07 0.032 1')
        palette = rng.choice(palettes)
        for crown in range(rng.randint(3, 5)):
            dx, dy = rng.uniform(-0.55, 0.55), rng.uniform(-0.55, 0.55)
            dz = height + rng.uniform(-0.15, 0.65)
            r = rng.uniform(0.55, 0.95)
            shade = rng.uniform(0.85, 1.12)
            color = ' '.join(f'{min(1, c*shade):.4f}' for c in palette) + ' 1'
            shape(link, 'visual', f'crown_cluster_{crown}', ('sphere', {'radius': f'{r:.4f}'}),
                  f'{dx:.4f} {dy:.4f} {dz:.4f} 0 0 0', color)
    shrubs = model(world, 'visual_shrubs')
    leaves = model(world, 'visual_leaf_piles')
    leaf_colors = ['0.88 0.58 0.055 1', '0.8 0.24 0.045 1', '0.57 0.08 0.035 1']
    leaf_index = 0
    for index, (x, y, radius, height) in enumerate(trees):
        a = rng.uniform(-math.pi, math.pi)
        sr = rng.uniform(0.2, 0.4)
        sx, sy = x + 0.65*math.cos(a), y + 0.65*math.sin(a)
        shape(shrubs, 'visual', f'shrub_{index:02}', ('sphere', {'radius': f'{sr:.4f}'}),
              f'{sx:.4f} {sy:.4f} {ground_z+sr*0.55:.4f} 0 0 0', rng.choice(['0.10 0.19 0.055 1', '0.29 0.27 0.06 1']))
        # Tight, overlapping clusters at several heights look like leaf piles.
        cx, cy = x + (radius+0.22)*math.cos(a), y + (radius+0.22)*math.sin(a)
        for layer in range(8):
            lx, ly = cx+rng.gauss(0, 0.09), cy+rng.gauss(0, 0.09)
            lz = ground_z + 0.003 + (layer % 3)*0.0035
            shape(leaves, 'visual', f'leaf_{leaf_index:03}', ('box', {'size': '0.1 0.1 0.005'}),
                  f'{lx:.4f} {ly:.4f} {lz:.4f} 0 0 {rng.uniform(-math.pi,math.pi):.4f}', rng.choice(leaf_colors))
            leaf_index += 1
    for _ in range(48):
        x, y = rng.uniform(-14, 14), rng.uniform(-14, 14)
        shape(leaves, 'visual', f'leaf_{leaf_index:03}', ('box', {'size': '0.1 0.1 0.005'}),
              f'{x:.4f} {y:.4f} {ground_z+0.003} 0 0 {rng.uniform(-math.pi,math.pi):.4f}', rng.choice(leaf_colors))
        leaf_index += 1
    package = Path(__file__).resolve().parents[1]
    E.indent(root, space='  ')
    E.ElementTree(root).write(package/'worlds/empty.sdf', encoding='utf-8', xml_declaration=True)
    rows = [f'seed: {seed}', f'ground_height: {ground_z}', 'obstacles:']
    rows.extend(f'  - {{x: {x:.5f}, y: {y:.5f}, radius: {r:.5f}, height: {h:.5f}}}' for x, y, r, h in trees)
    (package/'config/forest_map.yaml').write_text('\n'.join(rows)+'\n')
    print(f'Generated {count} random trees, {count} shrubs, {leaf_index} leaves; matching map saved')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', type=int, default=73)
    parser.add_argument('--trees', type=int, default=32)
    args = parser.parse_args()
    generate(args.seed, args.trees)
