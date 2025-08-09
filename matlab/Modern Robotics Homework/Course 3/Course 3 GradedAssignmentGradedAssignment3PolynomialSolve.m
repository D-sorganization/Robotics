syms t T A B C D E F
Polynomial=A*t^5+B*t^4+C*t^3+D*t^2+E*t+F:
dPolynomial=diff(Polynomial);
ddPolynomial=diff(dPolynomial)

PolynomialT=1==subs(Polynomial,t,1);
dPolynomialT=0==subs(dPolynomial,t,T);
ddPolynomialT=0==subs(ddPolynomial,t,T);
Polynomial0=subs(Polynomial,t,0);
dPolynomial0=subs(dPolynomial,t,0);
ddPolynomial0=subs(ddPolynomial,t,0);

constants=solve([PolynomialT dPolynomialT ddPolynomialT Polynomial0 dPolynomial0 ddPolynomial0],[A B C D E F])
A=constants.A;
B=constants.B;
C=constants.C;
D=constants.D;
E=constants.E;
F=constants.F;

Polynomial=A*t^5+B*t^4+C*t^3+D*t^2+E*t+F