clear;
clc;

// parâmetros
pi = 3.141592;
P.h_CG=0.72;                                        // altura do centro de gravidade [m]
P.h_CP=0.95;                                        // altura do centro de pressao [m]    
P.p_d=1.4;                                          // distância entre eixos [m]
P.p_CG=0.67;                                        // distância do CG ao eixo  traseiro [m]
P.m_m = 180;                                        // massa da moto [kg]
P.m_p = 70;                                         // massa da moto [kg]
P.k_A = 0.35;                                       // fator de arrasto aerodinamico [kg/m]
P.C_R = 0.02;                                       // fator de resistencia a rolagem 
P.m_c = P.m_m + P.m_p;                              // massa do conjunto [kg]
P.g = 9.81;                                         // aceleracao da gravidade [m/s^2]
P.mu = 0.8;                                         // coeficiente de atrito
P.W = -0;                                           // velocidade do vento [m/s]
P.teta = 15/180*pi;                                 // angulo de aclive da pista [rad]
P.P_max_roda_constante = 22080;                     // potencia constante na roda [W]
P.V=80/3.6;
P.alpha = 0.9367;                                   // acelerador
P.beta_t = 0;                                       // freio traseiro
P.beta_d = 0;                                       // dianteiro
E.Pmaxroda = P.P_max_roda_constante;                // potencia maxima constante 
P.R = 100000000;                                    // raio de curvatura

// calculo das forcas
E.Fgrx = -P.m_c*P.g*sin(P.teta);                    // componente x da gravidade
E.Fgry = -P.m_c*P.g*cos(P.teta);                    // componente y da gravidade
E.Fpro = P.alpha*E.Pmaxroda/P.V;                    // propulsao
E.Faer = -P.k_A*(P.V-P.W)^2;                        // arrasto aerodinamio
E.Frol = -P.C_R*abs(E.Fgry);                        // resistencia a rolagem

// cálculo da aceleração
E.a = (E.Fpro+E.Faer+E.Frol+E.Fgrx+P.beta_t*P.mu*E.Fgry + (P.beta_d-P.beta_t)*P.mu/P.p_d*( -E.Faer*P.h_CP + -E.Fgrx*P.h_CG + E.Fgry*P.p_CG)  )/(P.m_c*(1-(P.beta_d-P.beta_t)*P.mu*P.h_CG/P.p_d));

// Outras variáveis extraídas
E.Fine = -E.a*P.m_c;                                                                // inercia
E.Nrd = (E.Fine*P.h_CG + E.Faer*P.h_CP + E.Fgrx*P.h_CG - E.Fgry*P.p_CG)/P.p_d;      // normal dianteira
E.Nrt = -E.Nrd - E.Fgry;                                                            // normal traseira
E.Ffrt = -P.beta_t*P.mu*E.Nrt;                                                      // frenagem traseira
E.Ffrd = -P.beta_d*P.mu*E.Nrd;                                                      // frenagem dianteira
E.Fine = -P.m_c*E.a;                                                                // força de inércia
E.Fcd =  (P.m_c*P.V^2)/P.R*P.p_CG/P.p_d                                             // força centrípeta dianteira
E.Fct =  (P.m_c*P.V^2)/P.R*(1-P.p_CG/P.p_d)                                         // força centrípeta traseira
E.Fatd = sqrt(E.Ffrd^2 + E.Fcd^2)                                                   // força de atrito dianteira
E.Fatt = sqrt((E.Ffrt+E.Fpro)^2 + E.Fct^2)                                          // força de atrito traseira
E.Fatdisp_d = E.Nrd*P.mu                                                            // força de atrito disponível na dianteira    
E.Fatdisp_t = E.Nrt*P.mu                                                            // força de atrito disponível na traseira    
E.uadd = E.Fatd/(E.Nrd*P.mu)                                                        // utilização da aderência dianteira    c
E.uadt = E.Fatt/(E.Nrt*P.mu)                                                        // utilização da aderência traseira
E.Nestd = -E.Fgry*P.p_CG/P.p_d                                                      // normal dianteira estática
E.Nestt = -E.Fgry*(1-P.p_CG/P.p_d)                                                  // normal traseira estática
E.unod = 1 - E.Nrd/E.Nestd                                                          // utilização da normal dianteira
E.unot = 1 - E.Nrt/E.Nestt                                                          // utilização da normal traseira




