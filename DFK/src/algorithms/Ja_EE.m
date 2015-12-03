function Ja = Ja_EE(Jv,Jw,DmpB,AxesEuler,AnglesEuler)

B=BMatrix(AxesEuler,AnglesEuler,DmpB);
Jwa=B\Jw;
Ja=[Jv;Jwa];
end