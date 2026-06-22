#!/usr/bin/env python3
"""
gen_track_graph.py — Güncel track graf'ını (hedef_yoneticisi.build_track_graph)
map-server'ın okuyacağı statik YAML'a çıkarır.

NEDEN: hedef_yoneticisi grafı KODDA üretiyor (build_track_graph, ~644 node). Eski
Kerem imajındaki final_graph.yaml (60 node) ARTIK GEÇERSİZ. map-server'ı talos-all'a
taşırken /waypoint'in güncel grafı yayınlaması için bu script grafı maps/final_graph.yaml'a
döker. build_track_graph değişirse bu script'i tekrar çalıştır:

    cd scripts/talos26_ws && python3 maps/gen_track_graph.py

Çıktı: maps/final_graph.yaml  ->  {nodes: [{id,x,y}], edges: [[u,v]]}
"""
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)  # scripts/talos26_ws

# build_track_graph saf networkx/numpy/math kullanır; modül seviyesindeki ROS/matplotlib
# importlarını stub'layıp hedef_yoneticisi'ni import edebiliriz (HedefYoneticisi instantiate edilmez).
for m in ['rospy', 'matplotlib', 'matplotlib.pyplot',
          'std_msgs', 'std_msgs.msg', 'geometry_msgs', 'geometry_msgs.msg']:
    sys.modules[m] = types.ModuleType(m)
sys.modules['matplotlib'].pyplot = sys.modules['matplotlib.pyplot']
sys.modules['std_msgs.msg'].String = object
sys.modules['geometry_msgs.msg'].Pose2D = object

import importlib.util
import yaml

spec = importlib.util.spec_from_file_location(
    'hedef_yoneticisi', os.path.join(WS, 'hedef', 'hedef_yoneticisi.py'))
hy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hy)

G = hy.build_track_graph()

# İsimli node'ları (A1, laneB_3, ...) tamsayı id'ye eşle
name_to_id = {name: i for i, name in enumerate(G.nodes())}
nodes = []
for name, data in G.nodes(data=True):
    pos = data.get('pos')
    if pos is None:
        continue
    nodes.append({'id': name_to_id[name],
                  'x': round(float(pos[0]), 4),
                  'y': round(float(pos[1]), 4)})

edges = []
for u, v in G.edges():
    if u in name_to_id and v in name_to_id:
        edges.append([name_to_id[u], name_to_id[v]])

out = os.path.join(HERE, 'final_graph.yaml')
with open(out, 'w') as f:
    yaml.safe_dump({'nodes': nodes, 'edges': edges}, f, sort_keys=False)

print(f"OK: {out} yazildi — {len(nodes)} node, {len(edges)} edge "
      f"(kaynak: hedef/hedef_yoneticisi.build_track_graph)")
