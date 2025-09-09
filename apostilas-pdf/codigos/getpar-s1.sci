
function [FIS,MOT,PM,TRN] = get_parametros()
   
    // PARÂMETROS DA MOTO E DA PISTA
   
    FIS.h_CG=0.72;          // altura do centro de gravidade [m]
    FIS.h_CP=0.95;          // altura do centro de pressao [m]    
    FIS.p_d=1.4;            // distância entre eixos [m]
    FIS.p_CG=0.67;          // distância do CG ao eixo  traseiro [m]
  
    FIS.m_m = 180;          // massa da moto [kg]
    FIS.m_p = 70;           // massa da moto [kg]
    FIS.k_A = 0.35;         // fator de arrasto aerodinamico [kg/m]
    FIS.C_R = 0.02;         // fator de resistencia a rolagem 

    FIS.g = 9.81;           // aceleracao da gravidade [m/s^2]
    FIS.mu = 0.8;           // coeficiente de atrito
    FIS.W = -0;             // velocidade do vento [m/s]
    FIS.teta = 0/180*pi;    // angulo de aclive da pista [rad]
    
    // PARÂMETROS DO MOTOR
    
    MOT.P_pico = 22080/0.866;                 // Potência de pico [W]
    MOT.P_int = MOT.P_pico*0.15;              // Potência de diss. interna [W]
    MOT.P_corte_min = 3000                    // Potência na rotação min  [W]
    MOT.P_max_roda_constante = 22080;         // potencia constante na roda [W]
    
    MOT.w_motor_P_pico = 8000*(2*pi/60);         // rotaçao da potência de pico
    MOT.w_motor_T_pico = 4000*(2*pi/60);         // rotação do torque de pico
    MOT.w_motor_corte_min = 1500*(2*pi/60);      // rotação mínima
    MOT.w_motor_corte_max = 9000*(2*pi/60);      // rotação máxima
    
    MOT.eta_nominal = 0.40;                      // eficiência nominal do motor
    MOT.w_motor_eta_otimo = 5500*(2*pi/60);      // rotação de efic. máxima
    MOT.alpha_otimo = 0.9;                       // alpha ótimo
    MOT.eta_rot_rot_max = 0.9;                   // efic. eta_w na rot. máx
    MOT.eta_alpha_alpha_min = 0.9;                 // efic. eta_alpha na rot. min
    
    MOT.PCvol = 42500000*0.73*1000; // PC [J/m^3]
    
    // PARÂMETROS DA TRANSMISSÃO
    
    TRN.csi_prim = 2.8;                        // relação da trans. primária
    TRN.csi_marcha_i = [2.6 1.9 1.4 1.1 0.9];  // relação da caixa de marchas
    TRN.csi_fin = 2.6;                         // relação da trans. final
    
    TRN.RRoda = 0.3;                          //  raio da roda [m]
     
    TRN.eta_1 = 0.98;                         // eficiência da trans. prim               
    TRN.eta_2 = 0.98;                         // eficiência da embreagem              
    TRN.eta_3 = 0.98;                         // eficiência da caixa de marchas 
    TRN.eta_4 = 0.92;                         // eficiência da trans. final              
                                  
    TRN.eta_t = TRN.eta_1*TRN.eta_2*TRN.eta_3*TRN.eta_4;  
                                 // eficiência total da transmissão
    FIS.m_c = FIS.m_m + FIS.m_p;        // massa do conjunto [kg]
    PM = eval_parmot(MOT);              
                // cálculo dos coeficientes das curvas do motor
endfunction
