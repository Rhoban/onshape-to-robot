import numpy as np
import ast
import operator as op


class ExpressionParser:
    """
    Evaluate Onshape expression
    See https://cad.onshape.com/help/Content/numeric-fields.htm

    This parser is based on ast module.
    """

    def __init__(self):
        self.variables_lazy_loading = None
        self.variables = {
            "pi": np.pi,
        }

    # Supported operators
    operators = {
        ast.Add: op.add,
        ast.Sub: op.sub,
        ast.Mult: op.mul,
        ast.Div: op.truediv,
        ast.Pow: op.pow,
        ast.BitXor: op.xor,
        ast.USub: op.neg,
        ast.Mod: op.mod,
    }

    # Supporter functions
    functions = {
        "cos": np.cos,
        "sin": np.sin,
        "tan": np.tan,
        "acos": np.arccos,
        "asin": np.arcsin,
        "atan": np.arctan,
        "atan2": np.arctan2,
        "cosh": np.cosh,
        "sinh": np.sinh,
        "tanh": np.tanh,
        "asinh": np.arcsinh,
        "acosh": np.arccosh,
        "atanh": np.arctanh,
        "ceil": np.ceil,
        "floor": np.floor,
        "round": np.round,
        "exp": np.exp,
        "sqrt": np.sqrt,
        "abs": np.abs,
        "max": np.max,
        "min": np.min,
        "log": np.log,
        "log10": np.log10,
    }

    def eval_expr(self, expr):
        # Length units. Converting everything to meter / radian
        units = {
            "millimeter": 1e-3,
            "mm": 1e-3,
            "centimeter": 1e-2,
            "cm": 1e-2,
            "meter": 1.0,
            "inch": 0.0254,
            "in": 0.0254,
            "foot": 0.3048,
            "ft": 0.3048,
            "yard": 0.9144,
            "yd": 0.9144,
            "radian": 1.0,
            "rad": 1.0,
            "degree": np.pi / 180,
            "deg": np.pi / 180,
        }
        for unit, factor in units.items():
            expr = expr.replace(f" {unit}", f"*{factor}")

        expr = expr.replace("#", "")
        expr = expr.replace("^", "**")

        return self.eval_(ast.parse(expr, mode="eval").body)

    def eval_(self, node):
        if isinstance(node, ast.Constant):
            return float(node.value)
        elif isinstance(node, ast.BinOp):
            return self.operators[type(node.op)](
                self.eval_(node.left), self.eval_(node.right)
            )
        elif isinstance(node, ast.UnaryOp):  # e.g., -1
            return self.operators[type(node.op)](self.eval_(node.operand))
        elif isinstance(node, ast.Name):
            if (
                node.id.lower() not in self.variables
                and self.variables_lazy_loading is not None
            ):
                self.variables_lazy_loading()
                self.variables_lazy_loading = None
            if node.id.lower() not in self.variables:
                raise ValueError(f"Unknown variable in expression: {node.id}")
            return self.variables[node.id.lower()]
        elif isinstance(node, ast.Call):
            if node.func.id not in self.functions:
                raise ValueError(f"Unknown function in expression: {node.func.id}")
            return self.functions[node.func.id](*[self.eval_(arg) for arg in node.args])
        else:
            raise TypeError(node)


if __name__ == "__main__":
    ep = ExpressionParser()
    ep.variables["x"] = 5

    print(ep.eval_expr("(cos(5 deg)) mm + #x inch"))
    print(ep.eval_expr("-sin(3/(2^2) deg)"))
