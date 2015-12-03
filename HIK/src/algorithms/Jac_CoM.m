function JaCM = Jac_CoM(MCoM,T,nb,parent,axes,m,m_total)

JaCM = 0;

for i = 1:nb
Jv = Jg_EE(MCoM{i}(1:3,4),T,i,parent,axes);
Jv(:,2) = 0;
JaCM = JaCM + (Jv*m{i});
end

JaCM = JaCM/m_total;
end