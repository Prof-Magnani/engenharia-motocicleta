// ------------------------------------------------------/
// LÊ PARAMETROS DO PROBLEMA E CALCULA PARÂMETROS DERIVADOS
function [sP,sA] = get_parametros_v3()

    // --------------- DADOS --------------- 

    // PARÂMETROS NUMÉRICOS
    sP.npVseg=500;         // número de elementos do vetor de velocidade de segurança
    sP.dt = 0.1;           // incremento de tempo [s]
    sP.DT_num = 0.1;       // intervalo entre os instantes simulados
    sP.NumGd = 10000;      // número grande
    sP.NumPq = 0.001;      // número pequeno

    // DADOS DA SIMULAÇÃO
    sP.t_sim= 30;        // tempo simulado [s]
    sP.V0 = 1/3.6;        // velocidade inicial [m/s]
    sP.x0 = 0;            // posição inicial [s]
    sP.t0=0;              // tempo inicial [s]
    sP.Vref = 130/3.6;    // Velocidade de referência específica [km/h]

    // CARACTERÍSTICAS DA PISTA
    sP.Lpista = 500;             // comprimento total da pista [m]
    sP.L_tre1 = 250;             // comprimento do trecho 1 [m]
    sP.L_tre2 = 100               // comprimento do trecho 2 [m]
   sP.R_tre1 = sP.NumGd;        // raio de curvatura do trecho 1 [m]    
    sP.R_tre2 = 50         // raio de curvatura do trecho 2 [m]    
    sP.R_tre3 = 50         // raio de curvatura da curva 3 [m]

    // VELOCIDADE DESEJADA
    sP.V_ini_des = 1/3.6;     // velocidade inicial do vetor de velocidade desejada
    sP.L_Vvar_des = 200           // comprimento do trecho de velocidade variável no vetor de velocidade desejada
    sP.V_cruz_des = 200/3.6;        // velocidade de cruzeio do vetor de velocidade desejada
    
    // DADOS DO CONTROLE
    sP.u_alpha_max = 0.95;     // limite superior para o uso do controle de utilização do acelerador
    sP.u_beta_max = 0.95;      // limite superior para o uso do controle de utilização do freio  
    sP.dist_visao = 100;       // distância de visão do piloto
    sP.delta_alpha = 0.1;     // decremento do acelerador no controle de utilização
    sP.delta_beta = 0.1;      // decremento do freio no controle de utilização
    sP.f_red_Vseg = 0.9;      // fator de redução da velocidade de segurança

    // CONSTANTES GERAIS
    sP.pi = 3.141592;     // valor do pi
    sP.g = 9.81;          // aceleracao da gravidade [m/s^2]
    sP.R_bar = 8314.5;    // const. univ. gases [J/kmol.K]
    sP.T0 = 298;          // temperatura de referência, K   
   
    // PARÂMETROS DA MOTO
    sP.h_CG=0.72;          // altura do centro de gravidade [m]
    sP.h_CP=0.95;          // altura do centro de pressao [m]    
    sP.p_d=1.4;            // distância entre eixos [m]
    sP.p_CG=0.67;          // distância do CG ao eixo  traseiro [m]
    
    sP.m_m = 180;          // massa da moto [kg]
    sP.m_p = 70;           // massa da moto [kg]
    sP.k_A = 0.35;         // fator de arrasto aerodinamico [kg/m]
    sP.C_R = 0.02;         // fator de resistencia a rolagem 
    
    // PARÂMETROS DA PISTA
    sP.mu = 0.8;                    // coeficiente de atrito
    sP.W = -0;                      // velocidade do vento [m/s]
    sP.teta_pista = 0/180*sP.pi;    // angulo de aclive da pista [rad]
   
    // PARÂMETROS DO MOTOR
    sP.V_desl = 300/(1000*1000);                // volume deslocado [m3]
    sP.P_pico = 22080/0.866;                    // Potência de sP.pico [W]
    sP.P_corte_min = 3000                       // Potência na rotação min  [W]
    sP.P_max_roda_constante = 22080;            // potencia constante na roda [W]
    sP.w_motor_P_pico = 8000*(2*sP.pi/60);      // rotaçao da potência de sP.pico
    sP.w_motor_T_pico = 5500*(2*sP.pi/60);      // rotação do torque de sP.pico
    sP.w_motor_corte_min = 1500*(2*sP.pi/60);   // rotação mínima
    sP.w_motor_corte_max = 9000*(2*sP.pi/60);   // rotação máxima
    sP.r_DC = 1.1;                              // relação diâmetro/curso do pistão
    sP.r_MB = 1/4;                              // relação manivela/biela
    sP.eta_comp = 0.85;                         // eficiência de compressão
    sP.eta_exp = 0.85;                          // eficiência de compressão
    sP.T_refri = 95 + 273;                      // temperatura do fluido de resfriamento [K]]
    sP.U_refri = 1000;                          // coef. global. trans. cal. [W/m^2.K]
    sP.T_int_w_min = 1;                         // Torque de perda interna na rot. min, [N.m]
    sP.T_int_w_max = 3;                         // Torque de perda interna na rot. max, [N.m]
    
    // EFICIÊNCIA DO MOTOR
    sP.eta_nominal = 0.35;                      // eficiência nominal do motor
    sP.w_motor_eta_otimo = 5500*(2*sP.pi/60);   // rotação de efic. máxima
    sP.alpha_otimo = 0.9;                       // alpha no qual a eficiência é máxima
    sP.eta_rot_rot_max = 0.9;                   // efic. eta_w na rot. máx
    sP.eta_alpha_alpha_min = 0.9;               // efic. eta_alpha na rot. min
    
      
    // PARÂMETROS DA TRANSMISSÃO
    sP.csi_prim = 2.8;                        // relação da trans. primária
    sP.csi_marcha_i = [2.6 1.9 1.4 1.1 0.9];  // relação da caixa de marchas
    sP.csi_fin = 2.6;                         // relação da trans. final
    sP.RRoda = 0.3;                           //  raio da roda [m]
    sP.eta_1 = 0.98;                          // eficiência da trans. prim               
    sP.eta_2 = 0.98;                          // eficiência da embreagem              
    sP.eta_3 = 0.98;                          // eficiência da caixa de marchas 
    sP.eta_4 = 0.92;                          // eficiência da trans. final              

    // EFICIÊNCIA VOLUMÉTRICA
    sP.w_motor_eta_vol_otimo = 5500*(2*sP.pi/60);  // rotação de efic. vol. máx
    sP.eta_vol_max=1.15;                           // efic. vol. na rotação máxima
    sP.eta_vol_rot_min = 0.8;                      // efic. eta_w na rot. min

    // MISTURA ASPIRADA
    sP.sigma = 0.21;                         // percentual O2 no ar [kmol/kmol]
    sP.lambda = 1.0;                         // porcentual de ar teórico
    sP.p_amb = 101300;                       // pressão ambiente [Pa]
    sP.T_a = 25 + 273;                       // temperatura da carga pré-queima [K]
    
    // COMBUSTÍVEL
    sP.comb = 1;                             // 1: C8H18 ---- 2: C2H5OH  
    if sP.comb==1 then
        sP.at_x = 8;                         // átomos de C no combustível CxHyOz
        sP.at_y = 18;                        // átomos de H no combustível CxHyOz
        sP.at_z = 0;                         // átomos de O no combustível CxHyOz       
        sP.rho_comb = 730;                   // massa específica do combustível [kg/m3]
        sP.PC_comb = 42500000;               // poder calorífico [J/kg]
        sP.r_v = 9;                          // taxa de compressão
        sP.M(1) = sP.at_x*12 + sP.at_y*1 + sP.at_z*16;  //massa molar do combustível   ,   \si{kg/kmol} 
        sP.cv(1) = 1638*sP.M(1);                        // calor específico a volume constante, J/kg.k
        sP.uf(1) = -208600000 - sP.R_bar*sP.T0;         // energia interna de formação, J/kmol
    elseif sP.comb==2 then
        sP.at_x = 2;                         // átomos de C no combustível CxHyOz
        sP.at_y = 6;                         // átomos de H no combustível CxHyOz
        sP.at_z = 1;                         // átomos de O no combustível CxHyOz
        sP.rho_comb = 790;                   // massa específica do combustível [kg/m3]
        sP.PC_comb = 27000000;               // poder calorífico [J/kg]
        sP.r_v = 9.5;                        // taxa de compressão
        sP.M(1) = sP.at_x*12 + sP.at_y*1 + sP.at_z*16;   //massa molar do combustível   ,   \si{kg/kmol} 
        sP.cv(1)= 1246*sP.M(1);                          // calor específico a volume constante, J/kg.k
        sP.uf(1) = -235000000 - sP.R_bar*sP.T0;          // energia interna de formação J/kmol
    end
     
    // MOTOR ELÉTRICO
    sP.eta_el = 0.7;                            //eficiência do motor elétrico
    sP.rot_Trampa = 2000*(2*sP.pi/60);          // rotação do início da rampa, rad/s
    sP.rot_el_max = 9000*(2*sP.pi/60);          // rotação de corte do motor elétrico, rad/s
    sP.Trampa = 45;                             // torque do início da rampa, N.m
    sP.Twmax = 19;                              // torque na rotação máxima, N.m
   
    // PARÂMETROS DA COMBUSTÃO
    sP.nu_comb = 1;                             // coeficiente estequiométrico  do combustível
    sP.n_comp = 5;                              // número de componentes
    sP.teta_ini = 0;                            // ângulo inicial do virabrequim,     rad
    sP.teta_fim = 720*(sP.pi/180);              // ângulo final do virabrequim,     rad
    sP.teta_qp = (360-40)*(sP.pi/180);          // ângulo de início da queima principal, rad
    sP.delta_teta_queima = 100*(sP.pi/180);     // intervalo da queima principal, rad
    sP.delta_teta_delay_wref = 40*(sP.pi/180);  // interv. da queima de delay para $\omega_{\textit{ref}}$, rad
    sP.w_ref = 3600*(2*sP.pi/60);               // rotação de referência, rad/s
    sP.f_delay = 0.1;                           // percentual da queima no delay 
    sP.teta_a = 180*(sP.pi/180);                // angulo do início da compressão, rad
    sP.teta_b = 360*(sP.pi/180);                // angulo do início da expansão, rad
    sP.teta_c = 540*(sP.pi/180);                // angulo do início da descarga, rad
    sP.teta_d = 720*(sP.pi/180);                // angulo do início da admissão, rad

   // PROPRIEDADES DOS COMPONENTES
    sP.M(2) = 16*2;                                     //massa molar do O2   ,   \si{kg/kmol} 
    sP.M(3) = 14*2;                                     //massa molar do N2   ,   \si{kg/kmol} 
//    sP.M(4) = 12*1 + 16*2;                              //massa molar do CO2  ,   \si{kg/kmol}
    sP.M(5) = 1*2 + 16*1;                               //massa molar do H2O  ,   \si{kg/kmol}
    sP.cv(2) = 658*sP.M(2);                             //calor esp. vol. const. O2, \si{J/kg K}
    sP.cv(3) = 743*sP.M(3);                             //calor esp. vol. const. N2, \si{J/kg K}
    sP.cv(4) = 657*sP.M(4);                             //calor esp. vol. const. CO2, \si{J/kg K}
    sP.cv(5) = 1411*sP.M(5);                            //calor esp. vol. const. H2O, \si{J/kg K}
    sP.uf(2) = 0 - sP.R_bar*sP.T0;                      //energia interna de formação do O2 ,   \si{J/kmol}
    sP.uf(3) = 0 - sP.R_bar*sP.T0;                      //energia interna de formação do N2 ,   \si{J/kmol}
    sP.uf(4) = -393522000 - sP.R_bar*sP.T0;             //energia interna de formação do CO2 ,   \si{J/kmol}
    sP.uf(5) = -241826000 - sP.R_bar*sP.T0;             //energia interna de formação do H2O ,   \si{J/kmol} 
    
    // --------------- VARIÁVEIS AUXILIARES -------------- 

    // CÁLCULO DOS COEFICIENTES DOS POLINÔMIOS DA POTÊNCIA, EFICIÊNCIA DO MOTOR E EFICIÊNCIA VOLUMÉTRICA
    [sA.PMa,sA.PMb,sA.PMc,sA.PMd] = eval_parmot_v2(sP);             // cálculo dos coeficientes das curvas do motor

    // ESTEQUIOMETRIA
    sA.gamma = sP.at_x + sP.at_y/4 - sP.at_z/2;                                                     // gamma
    sA.AC_massa = sP.lambda* sA.gamma*(sP.sigma*sP.M(2) + (1-sP.sigma)*sP.M(3))/(sP.sigma*sP.M(1));    // relação ar-combustível mássica
    
    // CARACTERÍSTICAS DA MOTO
    sA.eta_t = sP.eta_1*sP.eta_2*sP.eta_3*sP.eta_4;  // eficiência total da transmissão
    sA.m_c = sP.m_m + sP.m_p;        // massa do conjunto [kg]
    
    // CARACTERÍSTICAS DO MECANISMO DO MOTOR
    sA.V_PMI = sP.V_desl*sP.r_v/(sP.r_v-1);                 // volume morto inferior, m3
    sA.V_PMS = sA.V_PMI/sP.r_v;                             // volume morto superior, m3
    sA.L_C = (4*sP.V_desl/(sP.pi*sP.r_DC^2))^(1/3);         // curso do pistao, m
    sA.D_pist = sA.L_C*sP.r_DC;                             // diâmetro do pistao, m
    sA.L_M = sA.L_C/2;                                      // comprimento da manivela, m
    sA.L_B = sA.L_M/sP.r_MB;                                // comprimento da biela, m
    sA.A_pist = (sP.pi*sA.D_pist^2)/4;                      // area do pistao, m2
    sA.A_cab = sA.A_pist;                                   // area do cabecote, m2
    sA.h_PMS = sA.V_PMS/sA.A_pist;                          // altura do cilindro no PMS, m
    sA.h_vbq =  sA.h_PMS +  sA.L_B + sA.L_M;                //distância do eixo do virabrequim ao cabeçote, m
 

   // COEFICIENTES ESTEQUIOMÉTRICOS DOS REAGENTES
    sA.nu(1,1)=sP.nu_comb;                                          // coeficiente estequiométrico do combustível                  
    sA.nu(1,2)=sP.nu_comb*sP.lambda*sA.gamma;                       // coeficiente estequiométrico do O2 nos reagentes
    sA.nu(1,3)=sP.nu_comb*sP.lambda*sA.gamma*(1-sP.sigma)/sP.sigma; // coeficiente estequiométrico do N2
    sA.nu(1,4)=0;                                      
    sA.nu(1,5)=0;                                      
    sA.soma_nu_reag = sum(sA.nu(1,:));
    
    // COEFICIENTES ESTEQUIOMÉTRICOS DOS PRODUTOS
    sA.nu(2,1)=0;
    sA.nu(2,2)=(sP.lambda-1)*sA.gamma*sP.nu_comb;
    sA.nu(2,3)=sP.nu_comb*sP.lambda*sA.gamma*(1-sP.sigma)/sP.sigma;         
    sA.nu(2,4)=sP.nu_comb*sP.at_x;
    sA.nu(2,5)=sP.nu_comb*sP.at_y/2;
    sA.soma_nu_prod = sum(sA.nu(2,:));
    
    // CALOR  ESPECÍFICO  E ENERGIA DE FORMAÇÃO MÉDIOS DOS REAGENTES
    sA.uf_reag = 0;
    sA.cv_reag = 0;
    sA.PC_comb_molar = 0;
    for i=1:sP.n_comp
        sA.yi_reag(i) = sA.nu(1,i)/sA.soma_nu_reag;
        sA.PC_comb_molar = sA.PC_comb_molar + (sA.nu(1,i)-sA.nu(2,i))/sA.nu(1,1)*(sP.uf(i)+sP.R_bar*sP.T0);
        sA.uf_reag = sA.uf_reag + sA.yi_reag(i)*sP.uf(i);
        sA.cv_reag = sA.cv_reag + sA.yi_reag(i)*sP.cv(i);
        sA.cp_reag = sA.cv_reag + sP.R_bar;
        sA.uf_est_reag = sA.uf_reag - sA.cv_reag*sP.T0;
    end
    sA.PC_comb_massa = sA.PC_comb_molar/sP.M(1);

    // CALOR  ESPECÍFICO  E ENERGIA DE FORMAÇÃO MÉDIOS DOS PRODUTOS
    sA.uf_prod = 0;
    sA.cv_prod = 0;
    for i=1:sP.n_comp
        sA.yi_prod(i) = sA.nu(2,i)/sA.soma_nu_prod;
        sA.uf_prod = sA.uf_prod + sA.yi_prod(i)*sP.uf(i);
        sA.cv_prod = sA.cv_prod + sA.yi_prod(i) *sP.cv(i);
        sA.cp_prod = sA.cv_prod + sP.R_bar;
        sA.uf_est_prod = sA.uf_prod - sA.cv_prod*sP.T0;
    end
       
endfunction
// ------------------------------------------------------/



