# Interoperability

**Table of Contents**
- [Interoperability](#interoperability)
  - [Python and Julia in harmony](#python-and-julia-in-harmony)
    - [Calling Julia code from Python code](#calling-julia-code-from-python-code)

## Python and Julia in harmony

Call Python code from Julia and Julia code from Python via a symmetric interface.
All details can be found in https://github.com/JuliaPy/PythonCall.jl.

### Calling Julia code from Python code

First, install the `juliacall` Python module with:
```bash
pip install juliacall
```

Then, in your Python script, import the `juliacall` module with:
```python
from juliacall import Main as jl
```

Next, navigate into the `interoperability` folder:
```bash
cd /path/to/TORA.jl/interoperability
```

Run the setup script to install the example dependencies:
```bash
python python_setup.py
```

Finally, run the example script with:
```bash
python python_calls_julia.py
```

Use `CTRL + C` at any point to stop running the example code.

The example script has comments explaining what is going on, and can be used as the basis for using the Julia framework from a Python environment for interfacing with a real robot (or with a simulator).
