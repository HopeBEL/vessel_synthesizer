# Blood-Vessel Synthesizer

This repository contains the implementation of the blood-vessel tree generator of our paper:   
:books: [Rauch N., Harders M., *Interactive Synthesis of 3D Geometries of Blood Vessels*, Eurographics 2021](https://diglib.eg.org/items/9e8cfe4d-b01a-4665-b3ab-f6e1857b40ee)   
:books: [Rauch N., Harders M., *Methods for User-Controlled Synthesis of Blood Vessel Trees in Medical Applications*, IEEE ACCESS](https://doi.org/10.1109/ACCESS.2025.3536998)

![example](img/example_banner.png)

The synthesizer emulates the abstract behavior of *angiogenesis* - a growth process in which blood vessels develop by elongating and branching from pre-existing vasculature to reach tissue devoid of any vasculature.
The overall mechanism resembles the competiton for space between individual branches, which we modeled with the *space colonization* algorithm.
During development the geometry of branches and bifurcations are further constrained to replicate commonly observed vascular patterns.

### Build

The library depends on [glm](https://github.com/g-truc/glm), [eigen](https://gitlab.com/libeigen/eigen), and [pybind11](https://github.com/pybind/pybind11) which are already contained.   

```bash
$ mkdir build && cd build/
$ cmake -DCMAKE_BUILD_TYPE=Release ..
$ cmake --build .
```

The synthesizer is implemented as a shared C++ library, with optional python-bindings (module).   
The following compile options are available:
| option    | description |
| --------  | -------     |
| VS_PYTHON_BINDINGS   | *build Python Bindings (module)*                                 |
| VS_PROFILER          | *build with Profiler Functionality (performance measurements)*   |
| VS_COMPILE_NATIVE    | *compile for micro-architecture and ISA extensions of the host*  |
| VS_COMPILE_FASTMATH  | *compile with fastmath optimization*                             |

and can be enabled during configuration:
```
cmake -DCMAKE_BUILD_TYPE=Release -DVS_PYTHON_BINDINGS=ON -DVS_PROFILER=ON ..
```

> ⚠️ library is not statically link against the c++ libraries; on windows you need to move the necessary .dll to the lib folder:
> * e.g. mingw (pthread) you need to add libgcc_s_seh-1.dll, libstdc++-6.dll, libwinpthread-1.dll

### Python Example

Check the notebook **notebook/api_showcase.ipynb** for python-module examples.

```python
# set path to compiled vessel module / library
import sys
sys.path.insert(0, '../build/lib/')
import vessel_module as vs

# define domain in which vessels should grow; can be = DomainCircle, DomainSphere, DomainLines, DomainVoxels
sphere = vs.DomainSphere([0.0, 0, 0], 0.5)

# create synthesizer object for domain
synth = vs.Synthesizer(sphere)

# modify settings (these are highly dependent on the dimensions of the domain, use scale to find an initial setup where things work)
synth.settings.steps = 100
synth.settings.samples = 1000
synth.settings.scale(1.5)

# set root note of tree (or use set_forest(...)) to grow from intial trees
synth.create_root(vs.System.ARTERIAL, [0.5, 0.0, 0.0])
synth.run()

```
![alt text](img/example_sphere.png)

```python
# check if library was compiled with performance monitor
if vs.PerfMonitor.Enabled:
    import pandas as pd
    import matplotlib.pyplot as plt

    # retrieve time measurements as dictonary
    times = synth.get_arterial_perftimes()

    # create pandas dataframe
    df = pd.DataFrame.from_dict(times)
    df = df.astype("timedelta64[ns]").astype("int64")
    df = df / 1e+6;

    # plot
    columns = ["total_arterial", "step", "step_closest", "step_growth", "step_kill"]
    fig, ax = plt.subplots(figsize=(10,5))
    df[columns].plot(ax = ax, kind="line", lw=2.5)
    df[df.columns.difference(columns)].plot(ax = ax, kind="area", stacked=True, alpha=0.75)

    ax.set_title("Performance Measurements Arterial Development")
    ax.set_xlabel("steps")
    ax.set_ylabel("time [ms]")
    ax.grid(which='major', color='#DDDDDD', linewidth=0.5)
```
![alt text](img/example_profile.png)

### Citation

If you use this code in your research, please cite one of our papers:

```bibtex
@inproceedings{10.2312:egs.20211012,
    booktitle = {Eurographics 2021 - Short Papers},
    title = {{Interactive Synthesis of 3D Geometries of Blood Vessels}},
    author = {Rauch, Nikolaus and Harders, Matthias},
    year = {2021},
    month = {May},
    publisher = {The Eurographics Association},
    ISSN = {1017-4656},
    ISBN = {978-3-03868-133-5},
    DOI = {10.2312/egs.20211012}
}
```


