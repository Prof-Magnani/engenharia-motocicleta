clear;                                  // limpa variáveis
clc;                                    // limpa a tela do console
getd('.\'); // carrega as funcões
[sP,sA] = get_parametros_v2();      // lê dados e calcula variáveis auxiliares
// parâmetros de cálculo
sR.rot_min = 1500*(2*sP.pi/60);     // rotação mínima
sR.rot_max = 9000*(2*sP.pi/60);     // rotação máxima
sR.NP = 200;                        // número de rotações calculadas
sR.ALPHAESTUDADO=1;                 // define o alpha estudado

// loop da rotação
for i=1:sR.NP
    sR.rot(i)=  (i-1)/(sR.NP-1)*(sR.rot_max-sR.rot_min) + sR.rot_min;                           // rotação
    sR.T_ref = sP.V_desl/(4*sP.pi)*sP.p_amb/(sP.R_bar*sP.T_a)*sP.sigma*sP.PC_comb*sP.M(1)/(sP.sigma + sP.lambda*sA.gamma);  // torque de referência
    sR.w_linha = (sR.rot(i)-sP.w_motor_corte_min)/(sP.w_motor_corte_max-sP.w_motor_corte_min);  // rotação normalizada
    sR.P_int(i) = sR.rot(i)*(sR.w_linha*(sP.T_int_w_max - sP.T_int_w_min) + sP.T_int_w_min);     // potência dissipada interna

    // cálculos para alpha = 1
    ALPHA1=1;                                                                                   // uso do acelerador
    sR.eta_m_Pmax(i) = eval_efic (sP,sA,sR.rot(i),ALPHA1);                                      // eficiência do motor
    sR.eta_vol_max(i) = sA.PMd(1) + sA.PMd(2)*sR.w_linha + sA.PMd(3)*sR.w_linha^2;              // eficiência volumétrica máxima
    sR.Pmax_motor(i) = sR.eta_m_Pmax(i)*sR.eta_vol_max(i)*sR.rot(i)*sR.T_ref - sR.P_int(i);      // potência máxima do motor         

    // cálculos para alpha genérico
    sR.alpha(i) =sR.ALPHAESTUDADO                                                                              // uso do acelerador
    sR.eta_m(i) = eval_efic (sP,sA,sR.rot(i),sR.alpha(i));                                      // eficiência do motor
    sR.eta_vol(i) = sR.alpha(i)*sR.eta_m_Pmax(i)/sR.eta_m(i)*sR.eta_vol_max(i) + (1-sR.alpha(i))*sR.P_int(i)/(sR.eta_m(i)*sR.rot(i)*sR.T_ref); // eficência volumétrica
    sR.P_motor(i) = sR.eta_m(i)*sR.eta_vol(i)*sR.rot(i)*sR.T_ref - sR.P_int(i);                 // potência do motor [W]
    
    // variáveis adicionais
    sR.p_a(i) = sP.p_amb*sR.eta_vol(i);                                                          // pressão mistura aspirada
    sR.tau(i) = sP.sigma/(sP.sigma + sP.lambda*sA.gamma)*sR.p_a(i)*sP.M(1)/(sP.R_bar*sP.T_a);    // massa combustível por volume da mistura   

    // motor empírico
    sR.P_motor_emp(i)=eval_Pmotmax(sP,sA,sR.rot(i));                                             // potência do motor empírico

    // zera variáveis abaixo e acima dos limites de corte
    if (sR.rot(i)>sP.w_motor_corte_max) || (sR.rot(i)<sP.w_motor_corte_min)
        sR.P_motor(i) = 0;
        sR.P_motor_emp(i) = 0;
        sR.eta_m_Pmax(i) = 0;
        sR.P_int(i) = 0;
    end    

    // motor elétrico
    if sR.rot(i)<sP.rot_Trampa                          
        sR.T_el = sP.Trampa;                                                            // torque constante abaixo da rotação da rampa
    elseif sR.rot(i)<=sP.rot_el_max
        rot_esp = (sR.rot(i)-sP.rot_Trampa)/(sP.rot_el_max-sP.rot_Trampa)               // rotação específica
        sR.T_el = rot_esp*(sP.Twmax - sP.Trampa) + sP.Trampa;                           // torque linear na rampa
    else
        sR.T_el = 0;                                                                    // torque nulo acima da rotação máxima
    end
    sR.P_el(i) = sR.T_el*sR.rot(i);                                                     // potência = torque x rotação          
end 

// gráficos
//plot(sR.rot/(2*sP.pi/60),sR.P_motor/736,'k','LineWidth',3)
//plot(sR.rot/(2*sP.pi/60),sR.P_motor_emp/736,'k','LineWidth',3)
//plot(sR.rot/(2*sP.pi/60),sR.P_int/736,'k','LineWidth',3)
//plot(sR.rot/(2*sP.pi/60),sR.P_el/736,'k','LineWidth',3)
//plot(sR.rot/(2*sP.pi/60),sR.eta_vol,'k','LineWidth',3)
//plot(sR.rot/(2*sP.pi/60),sR.eta_m,'k','LineWidth',3)
//plot(sR.rot/(2*sP.pi/60),sR.P_motor./sR.rot/9.81,'k','LineWidth',3)
//plot(sR.rot/(2*sP.pi/60),sR.P_el./sR.rot/9.81,'k','LineWidth',3)
//xlabel('$\omega\quad[rpm]$','fontsize',4);       // titulo do eixo x
//ylabel('$P_{motor}\quad[CV]$','fontsize',4);    // titulo do eixo y)
//ylabel('$T_{motor}\quad[kgf.m]$','fontsize',4);    // titulo do eixo y)
//ylabel('$\eta_{vol}$','fontsize',4);    // titulo do eixo y)
//ylabel('$\eta_{m}$','fontsize',4);    // titulo do eixo y)
//h = gca(); // get current axes
//h.data_bounds = [1000, 0.32 ; 9000,0.41];
//h.tight_limits=["on","off"];
//disp(sR.rot(i)/(2*sP.pi/60))
//disp(sR.P_motor(i))
