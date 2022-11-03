from sympy import false
from utils import *

def main ():

    eyebrow_down()
    while(1):

        if have_to_go():
            eyebrow_up()
            break

main()
