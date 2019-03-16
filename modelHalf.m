function dZ_2 = modelHalf(t, Z_2, SimulationParams)

p = SimulationParams.SuspensionParams;
actuationTime = SimulationParams.actuationTime;
actuationFun = SimulationParams.actuationFun;
actuationVals = SimulationParams.actuationVals;

Z_0 = actuationFun(actuationVals, t, actuationTime);

A = [0, 1, 0, 0, 0, 0, 0, 0;
     -(p.k_1 + p.k_3)/p.m_1, -(p.C_1 + p.C_3)/p.m_1, 0, 0, p.k_3/p.m_1, p.C_3/p.m_1, -p.k_3*p.l_F/p.m_1 -p.C_3*p.l_F/p.m_1;
     0, 0, 0, 1, 0, 0, 0, 0;
     0, 0, -(p.k_2 + p.k_4)/p.m_2, -(p.C_2 + p.C_4)/p.m_2, p.k_4/p.m_2, p.C_4/p.m_2, p.k_4*p.l_R/p.m_2 p.C_4*p.l_R/p.m_2;
     0, 0, 0, 0, 0, 1, 0, 0;
     p.k_3/p.m_s, p.C_3/p.m_s, p.k_4/p.m_s, p.C_4/p.m_s, -(p.k_3 + p.k_4)/p.m_s, -(p.C_3 + p.C_4)/p.m_s, -(p.k_4*p.l_R - p.k_3*p.l_F)/p.m_s, -(p.C_4*p.l_R - p.C_3*p.l_F)/p.m_s;
     0, 0, 0, 0, 0, 0, 0, 1;
     -(p.l_F*p.k_3)/p.I_s, -(p.l_F*p.C_3)/p.I_s, (p.l_R*p.k_4)/p.I_s, (p.l_R*p.C_4)/p.I_s, -(p.l_R*p.k_4 - p.l_F*p.k_3)/p.I_s, -(p.l_R*p.C_4 - p.l_F*p.C_3)/p.I_s, -((p.l_R^2)*p.k_4 + (p.l_F^2)*p.k_3)/p.I_s, -((p.l_R^2)*p.C_4 + (p.l_F^2)*p.C_3)/p.I_s];

b = [0, 0, 0, 0;
     p.k_1/p.m_1, p.C_1/p.m_1, 0, 0;
     0, 0, 0, 0;
     0, 0, p.k_2/p.m_2, p.C_2/p.m_2;
     0, 0, 0, 0;
     0, 0, 0, 0;
     0, 0, 0, 0;
     0, 0, 0, 0];

dZ_2 = A*Z_2 + b*Z_0';

end

