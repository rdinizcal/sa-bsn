import re

class Formula:

    def __init__(self, path, _type): 
        self.type = _type
        formula_file = open(path, 'r')
        if formula_file.mode == 'r': 
            self.expression = formula_file.read()
            self.mapping = self.initialize_expr()
        else : 
            raise Exception('Formula file not found')
    
    def initialize_expr(self):
        expr = self.expression.replace("*"," ")
        expr = expr.replace("+"," ")
        expr = expr.replace("-"," ")
        expr = expr.replace("/"," ")
        expr = expr.replace("("," ")
        expr = expr.replace(")"," ")
        expr = expr.replace("and"," ")
        expr = expr.replace("or"," ")
        expr = re.split(' ',expr)
        arguments = list(filter(None, expr))

        arg_val = {}
        for argument in arguments :
            if self.type == "bool": arg_val[argument] = False
            else : arg_val[argument] = 0

        return arg_val

    def compute(self, arg, value):
        if self.type == "bool": self.mapping[arg] = bool(value)
        else: self.mapping[arg] = float(value)
    
    def eval(self):
        return eval(self.expression, self.mapping)

formula = Formula("./resource/models/reliability.formula", "float")

formula.compute("CTX_G3_T1_1", 1)
formula.compute("F_G3_T1_1", 1)
formula.compute("R_G3_T1_1", 0.92)
formula.compute("CTX_G3_T1_2", 1)
formula.compute("F_G3_T1_2", 1)
formula.compute("R_G3_T1_2", 0.92)
formula.compute("CTX_G3_T1_3", 1)
formula.compute("F_G3_T1_3", 1)
formula.compute("R_G3_T1_3", 0.92)
formula.compute("CTX_G3_T1_4", 1)
formula.compute("F_G3_T1_4", 1)
formula.compute("R_G3_T1_4", 1)
formula.compute("CTX_G4_T1", 1)
formula.compute("F_G4_T1", 1)
formula.compute("R_G4_T1", 0.92)

R_G3_T1_4=0
F_G3_T1_4=0
CTX_G3_T1_4=0
R_G4_T1=0.92
F_G4_T1=1
CTX_G4_T1=1


print(formula.eval())