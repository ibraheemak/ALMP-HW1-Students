"""
Run all tests from a JSON file against the HW1 module and produce plots.

Usage:
  python run_all_tests.py [path/to/tests.json]

Behavior:
  - Loads tests from the provided JSON file (default: "tests.json").
  - Tries to import module named "HW1" or "hw1".
  - Looks for one of these callables inside HW1: run_test, solve, plan, main.
    It will inspect the callable signature and attempt to call it with a single
    test dict where possible. If the callable expects no args, it will call it once.
  - For each test, an output PNG plot is created under ./results/.
"""

# ...existing code...

import os
import sys
import json
import importlib
import inspect
import traceback
from typing import Any, Dict, List, Optional

# plotting helpers
import matplotlib.pyplot as plt

def load_tests(path: str) -> List[Dict[str, Any]]:
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    if isinstance(data, list):
        return data
    if isinstance(data, dict) and 'tests' in data and isinstance(data['tests'], list):
        return data['tests']
    # fallback: try to find list-valued keys
    for v in data.values() if isinstance(data, dict) else []:
        if isinstance(v, list):
            return v
    raise ValueError("Unable to interpret JSON file as list of tests. Expect list or {'tests': [...]}")

def import_hw1_module() -> Any:
    for name in ('HW1', 'hw1', 'Hw1'):
        try:
            return importlib.import_module(name)
        except Exception:
            pass
    raise ImportError("Could not import HW1 module. Please ensure file HW1.py (or hw1.py) is on PYTHONPATH.")

def find_callable(module: Any):
    candidates = ['run_test', 'solve', 'plan', 'main']
    # 1) Prefer explicit candidate names first if they are functions (not classes)
    for name in candidates:
        fn = getattr(module, name, None)
        if fn is not None and callable(fn) and not inspect.isclass(fn):
            return name, fn

    # 2) Gather all non-class callables defined in this module and score them by heuristics
    def score_fn(name, fn):
        score = 0
        try:
            if inspect.isclass(fn):
                return -1000  # strongly ignore classes
            # prefer functions defined in the same module
            if getattr(fn, "__module__", None) == module.__name__:
                score += 5
            # prefer callables whose parameter names indicate they accept a test dict
            try:
                params = list(inspect.signature(fn).parameters.keys())
            except Exception:
                params = []
            for p in params:
                if p.lower() in ('test', 'tests', 'case', 'case_data', 'cfg', 'config', 'problem', 'spec'):
                    score += 4
                if p.lower() in ('path', 'points', 'coords', 'coordinates', 'pts'):
                    score += 1
            # small bonus if the attribute name looks like a runner
            if name.lower() in ('run', 'run_test', 'execute', 'solve', 'plan', 'main'):
                score += 2
        except Exception:
            return 0
        return score

    candidates_list = []
    for attr in dir(module):
        if attr.startswith('_'):
            continue
        fn = getattr(module, attr)
        if callable(fn):
            s = score_fn(attr, fn)
            candidates_list.append((s, attr, fn))
    candidates_list.sort(reverse=True, key=lambda x: x[0])

    # choose the highest-scoring callable if it's reasonable
    if candidates_list and candidates_list[0][0] > 0:
        return candidates_list[0][1], candidates_list[0][2]

    # 3) Fallback: pick any non-class callable
    for attr in dir(module):
        if attr.startswith('_'):
            continue
        fn = getattr(module, attr)
        if callable(fn) and not inspect.isclass(fn):
            return attr, fn
    raise AttributeError("No suitable callable found in HW1 module. Please provide run_test/test_runner/solve/plan/main.")

def call_test_fn(fn, test: Dict[str, Any], idx: int):
    try:
        sig = inspect.signature(fn)
    except Exception:
        sig = None

    try:
        if sig is None or len(sig.parameters) == 0:
            # expects no args
            return fn()
        elif len(sig.parameters) == 1:
            # Decide what the single parameter likely expects by name
            param_name = next(iter(sig.parameters.keys()))
            pname = param_name.lower()
            # If the parameter name suggests it expects a test-like dict, pass the test dict
            test_like_params = ('test', 'tests', 'case', 'case_data', 'cfg', 'config', 'problem', 'spec')
            seq_like_params = ('path', 'points', 'coords', 'coordinates', 'pts', 'trajectory', 'traj', 'waypoints')

            if pname in test_like_params:
                return fn(test)
            if pname in seq_like_params:
                # try extracting that sequence from the test dict
                candidate = test.get(param_name) or test.get('path') or test.get('points') or test.get('coords') or test.get('trajectory') or test.get('traj')
                if candidate is not None:
                    return fn(candidate)
                # if not found, try passing any list-like content from test
                for k, v in test.items():
                    if isinstance(v, (list, tuple)):
                        return fn(v)
                # last resort: try passing the whole dict (some APIs may accept it)
                return fn(test)
            # unknown parameter name: attempt kwargs first, then positional test
            try:
                return fn(**test)
            except Exception:
                return fn(test)
        else:
            # multi-parameter: try matching by kwargs
            try:
                return fn(**test)
            except Exception:
                # fallback: pass the entire test dict as first arg
                return fn(test)
    except Exception:
        # provide traceback but continue
        print(f"Error running test #{idx}:")
        traceback.print_exc()
        return None

# simple plotting function: accepts a path as list of (x,y) or ndarray
def plot_path(path: Optional[List], obstacles: Optional[List]=None, start=None, goal=None, title=None, save_path: Optional[str]=None, show: bool=False):
    plt.figure(figsize=(6,6))
    if obstacles:
        # expect obstacles as list of dicts with 'x','y','r' or polygons -- try best-effort
        for obs in obstacles:
            try:
                if 'r' in obs and ('x' in obs or 'cx' in obs):
                    cx = obs.get('x', obs.get('cx'))
                    cy = obs.get('y', obs.get('cy'))
                    r = obs.get('r')
                    circle = plt.Circle((cx, cy), r, color='gray', alpha=0.5)
                    plt.gca().add_patch(circle)
                elif isinstance(obs.get('pts', None), list):
                    pts = obs['pts']
                    xs = [p[0] for p in pts]
                    ys = [p[1] for p in pts]
                    plt.fill(xs, ys, color='gray', alpha=0.5)
            except Exception:
                continue
    if path:
        try:
            xs = [p[0] for p in path]
            ys = [p[1] for p in path]
            plt.plot(xs, ys, '-o', color='blue', label='path')
        except Exception:
            # try interpretting path as list of dicts
            try:
                xs = [p['x'] for p in path]
                ys = [p['y'] for p in path]
                plt.plot(xs, ys, '-o', color='blue', label='path')
            except Exception:
                pass
    if start:
        try:
            plt.plot(start[0], start[1], 'go', markersize=8, label='start')
        except Exception:
            pass
    if goal:
        try:
            plt.plot(goal[0], goal[1], 'ro', markersize=8, label='goal')
        except Exception:
            pass
    if title:
        plt.title(title)
    plt.axis('equal')
    plt.legend()
    if save_path:
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        plt.savefig(save_path, bbox_inches='tight')
        print(f"Saved plot to {save_path}")
    if show:
        plt.show()
    plt.close()

def ensure_results_dir(base='results'):
    os.makedirs(base, exist_ok=True)
    return base

def try_extract_path_from_result(result):
    # heuristics: common result shapes
    if result is None:
        return None
    # if result contains key 'path' or 'trajectory'
    if isinstance(result, dict):
        for key in ('path', 'trajectory', 'traj', 'solution'):
            if key in result:
                return result[key]
    # if result looks like a sequence of points
    if isinstance(result, (list, tuple)):
        if len(result) > 0 and (isinstance(result[0], (list, tuple)) or isinstance(result[0], dict)):
            return result
    return None

def main(argv):
    json_path = argv[1] if len(argv) > 1 else 'tests.json'
    # if user passed --no-show, disable showing; otherwise show by default to match HW1 behavior
    show = '--no-show' not in argv and '-q' not in argv

    if not os.path.exists(json_path):
        print(f"JSON tests file not found: {json_path}")
        return 1

    tests = load_tests(json_path)
    print(f"Loaded {len(tests)} tests from {json_path}")

    module = import_hw1_module()
    name, fn = find_callable(module)
    print(f"Using HW1 callable: {name}")

    results_dir = ensure_results_dir('results')

    # Try to import Plotter class
    try:
        from Plotter import Plotter
    except ImportError:
        try:
            # Try getting it from the module if it's there
            Plotter = getattr(module, 'Plotter', None)
        except Exception:
            Plotter = None

    for i, test in enumerate(tests):
        print(f"Running test #{i}: {test.get('name', test.get('id', str(i)))}")
        result = call_test_fn(fn, test, i)

        # Use Plotter class if available and result has the right structure
        plotted = False
        if Plotter is not None and isinstance(result, dict):
            try:
                # Extract data from result (as returned by run_test)
                workspace_obstacles = result.get('workspace_obstacles')
                c_space_obstacles = result.get('c_space_obstacles')
                visibility_graph = result.get('visibility_graph')
                path = result.get('path')
                source = result.get('source')
                dest = result.get('dest')
                dist = result.get('dist')
                
                # If we have the data, use Plotter like HW1 does
                if workspace_obstacles is not None and c_space_obstacles is not None:
                    plotter = Plotter()
                    plotter.add_obstacles(workspace_obstacles)
                    plotter.add_c_space_obstacles(c_space_obstacles)
                    
                    if visibility_graph is not None:
                        plotter.add_visibility_graph(visibility_graph)
                    
                    if source is not None and dist is not None:
                        plotter.add_robot(source, dist)
                    
                    if dest is not None and dist is not None:
                        plotter.add_robot(dest, dist)
                    
                    if path is not None and len(path) > 0:
                        plotter.add_shorterst_path(path)
                    
                    # Display the plot like HW1 does
                    plotter.show_graph()
                    plotted = True
            except Exception as e:
                print(f"Error using Plotter: {e}")
                plotted = False

        if not plotted:
            # Fallback to simple plotting
            path = try_extract_path_from_result(result)
            if path is None and isinstance(result, tuple):
                for item in result:
                    p = try_extract_path_from_result(item)
                    if p is not None:
                        path = p
                        break

            # Fallback: Peek into test to find start/goal/obstacles to plot
            start = test.get('start') or test.get('s') or test.get('init')
            goal = test.get('goal') or test.get('g') or test.get('target')
            obstacles = test.get('obstacles') or test.get('obs')

            # Use local plotting helper; just display
            plot_path(path=path, obstacles=obstacles, start=start, goal=goal, title=f"test_{i}", save_path=None, show=True)

    print("All tests processed.")
    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv))
