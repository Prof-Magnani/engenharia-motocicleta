clear;
clc;

// VEÍCULO
//1: índice da moto média
//2: índice do carro
//3: índice da moto pequena
//4: índice da moto elétrica
//5: índice da bicicleta
//6: índice do mototaxi
//7: índice do onibus

// MÉTRICA
//1: Cp
//2: Op
//3: Tg
//4: CPS
//5: glb,f
//6: glb,ft
//7: glb,fts
//8: ee
//9: cc

// TIPOS DE ESTUDO
//1: variação do deslocamento mensal
//2: variação do salário 

sP.NSA = 0                  // não se aplica
sP.estudo = 2               // estudo mostrado no gráfico
sP.metrica = 5              // métrica mostrada no gráfico
sP.S_min = 0*1000;          // salário mínimo mostrado no gráfico
sP.S_max = 12*1000;         // salário máximo mostrado no gráfico
sP.L_min = 0*1000*30;       // distância diária mínima mostrada no gráfico
sP.L_max = 50*1000*30;      // distância diária máxima mostrada no gráfico

sP.S_CB = 6000;             // salário do caso base
sP.L_dia_CB = 30            // distância diária do caso base
sP.pi = 3.151592;           // valor do pi
sP.g = 9.81;                // valor da gravidade
sP.n_veic = 7;              // número de tipos de veículos
sP.dias_mes = 30            // número de dias no mês

sP.L_mes = sP.L_dia_CB*sP.dias_mes*1000*ones(sP.n_veic,1);  // distância percorrida por mês
sP.S_mes = sP.S_CB*ones(sP.n_veic,1);                       // salário mensal
sP.L_pst = 470*ones(sP.n_veic,1);                           // comprimento de cada pista
sP.theta_sP.pista = 0*(sP.pi/180)*ones(sP.n_veic,1);        // aclive da pista
sP.eta_t = 0.866*ones(sP.n_veic,1);                         // eficiência da transmissão
sP.n_pas = 1*ones(sP.n_veic,1);                             // número de passageiros
sP.m_pas = 80*ones(sP.n_veic,1);                            // massa de cada passageiro
sP.m_crg = 0*ones(sP.n_veic,1);                             // massa da carga
sP.rho_comb = 730*ones(sP.n_veic,1);                        // massa específica do combustível
sP.PC_comb = 42.5e6*ones(sP.n_veic,1);                      // poder calorífico do combustível
sP.n_mes = 48*ones(sP.n_veic,1);                            // número de meses
sP.f_j = 0.01*ones(sP.n_veic,1);                            // juros por mês
sP.V_ref = 5/3.6*ones(sP.n_veic,1);                         // velocidade de referência  
sP.f_cmpr = 0.3                                             // fator de comprometimento

sP.a_rmp = [0.54, 0.30, 0.15, 0.54, 0.05, sP.NSA, sP.NSA];      // aceleração da rampa
sP.V_crz = [42, 25, 20, 42, 10, 42, 15]/3.6;                    // velocidade de cruzeiro
sP.delta_t_est_mes = [120, 240, 60, 120, 30, 120, 480]*60;      // tempo mensal no estacionamento
sP.delta_t_sem_pst = [30, 90, 30, 30, 30, 30, 120];             // tempo no semáforo por pista
sP.m_vei = [170, 1000, 80, 220, 15, sP.NSA, sP.NSA];            // massa do veículo
sP.C_R = [0.02, 0.06, 0.02, 0.02, 0.02, sP.NSA, sP.NSA];        // coeficiente de rolagem
sP.k_A = [0.35, 0.70, 0.30, 0.35, 0.20, sP.NSA, sP.NSA];        // fator de arrasto
sP.eta_m = [0.315, 0.315, 0.315, 0.800, 0.315, sP.NSA, sP.NSA]; // eficiência do motor
sP.P_int = [1370, 3000, 400, 1000, 50, sP.NSA, sP.NSA];         // potência interna dissipada
sP.I_compra = [15000, 40000, 5000, 25000, 500, 0, 0];           // valor de compra
sP.I_revenda = [6000, 20000, 1000, 10000, 100, 0, 0];           // valor de revenda
sP.f_oper = [1.2, 1.2, 1.1, 1.2, 1.05, sP.NSA, sP.NSA];         // fator operacional
sP.f_compra = [1.3, 1.3, 1.1, 1.3, 1.05, sP.NSA, sP.NSA];       // fator de compra 
sP.f_pt = 1*[+1, +4, +1, +1, +1, +1, +0]/1000;                  // fator de prestígio
sP.f_tg = 10*[+1, +1, +1, +1, +1, +1, +0]/1000;                 // fator de tempo ganho
sP.f_cf = 1*[+1, +2, +1, +1, +0, +1, +1]/1000;                  // fator de conforto
sP.f_sg = 1*[-1, +1, -1, -1, +0, -1, +1]/1000;                  // fator de segurança

sP.kappa_comb = [6.2, 6.2, 6.2, 0.85*sP.PC_comb(4)/(1000*3600), 17.5*sP.PC_comb(5)/11103500, sP.NSA , sP.NSA];      // tarifa do combustível

// LOOP DOS ESTUDO
sP.NP = 300
for fat=1:sP.NP
    par_fat(fat) = (fat-1)/(sP.NP-1)
    // LOOP DOS VEÍCULOS
    for i=1:sP.n_veic
        if sP.estudo==1 then                // se o estudo for de variação da distância diária
            sP.val_fat(fat) = sP.L_min + par_fat(fat)*(sP.L_max-sP.L_min)
            sP.L_mes(i) = sP.val_fat(fat);
        else                                // se o estudo for de variação do salário
            sP.val_fat(fat) = sP.S_min + par_fat(fat)*(sP.S_max-sP.S_min)
            sP.S_mes(i) = sP.val_fat(fat);
        end    

        // EQUAÇÕES DA APOSTILA
        sR.delta_t_rmp(i) = sP.V_crz(i)/(sP.a_rmp(i));
        sR.L_rmp(i) = (sP.V_crz(i)^2)/(2*sP.a_rmp(i));
        sR.L_crz(i)= sP.L_pst(i) - sR.L_rmp(i) ; 
        sR.delta_t_crz(i)= sP.L_pst(i)/sP.V_crz(i) - sP.V_crz(i)/(2*sP.a_rmp(i)); 
        sR.delta_t_pst(i)= sR.delta_t_rmp(i) + sR.delta_t_crz(i); 
        sR.m_uti(i)= sP.n_pas(i)*sP.m_pas(i) + sP.m_crg(i); 
        sR.m_tot(i)= sP.m_vei(i) + sR.m_uti(i); 
        sR.P_pro_fim_rmp(i)= sR.m_tot(i)*sP.a_rmp(i)*sP.V_crz(i) + sP.C_R(i)*sR.m_tot(i)*sP.g*cos(sP.theta_sP.pista(i))*sP.V_crz(i) + sR.m_tot(i)*sP.g*sin(sP.theta_sP.pista(i))*sP.V_crz(i) + sP.k_A(i)*sP.V_crz(i)^3; 
        sR.P_pro_crz(i)= sP.C_R(i)*sR.m_tot(i)*sP.g*cos(sP.theta_sP.pista(i))*sP.V_crz(i) + sR.m_tot(i)*sP.g*sin(sP.theta_sP.pista(i))*sP.V_crz(i) + sP.k_A(i)*sP.V_crz(i)^3; 
        sR.E_pro_rmp(i)= sR.m_tot(i)*(sP.V_crz(i)^2)/(2) + sP.C_R(i)*sR.m_tot(i)*sP.g*cos(sP.theta_sP.pista(i))*(sP.V_crz(i)^2)/(2*sP.a_rmp(i)) + sR.m_tot(i)*sP.g*sin(sP.theta_sP.pista(i))*(sP.V_crz(i)^2)/(2*sP.a_rmp(i)) + sP.k_A(i)*(sP.V_crz(i)^4)/(4*sP.a_rmp(i));    
        sR.E_pro_crz(i)= (sP.C_R(i)*sR.m_tot(i)*sP.g*cos(sP.theta_sP.pista(i)) + sR.m_tot(i)*sP.g*sin(sP.theta_sP.pista(i)) + sP.k_A(i)*sP.V_crz(i)^2)*(sP.L_pst(i) - (sP.V_crz(i)^2)/(2*sP.a_rmp(i))); 
        sR.E_pro_pst(i)=  sR.E_pro_rmp(i) + sR.E_pro_crz(i);
        sR.E_font_pst(i)= sR.E_pro_pst(i)/(sP.eta_t(i)*sP.eta_m(i)) + sP.P_int(i)*sR.delta_t_pst(i)/sP.eta_m(i); 
        sR.m_comb_pst(i)= sR.E_font_pst(i)/sP.PC_comb(i); 
        sR.VV_pst(i)= sR.E_font_pst(i)/(sP.rho_comb(i)*sP.PC_comb(i)); 
        sR.mu_mp(i)= ((1+sP.f_j(i))^sP.n_mes(i)-1)/(sP.f_j(i)*(1+sP.f_j(i))^sP.n_mes(i)); 
        sR.mu_pm(i)= 1/sR.mu_mp(i);
        sR.mu_f_nmes_p(i) = 1/((1+sP.f_j(i))^sP.n_mes(i))
        sR.mu_p_f_nmes(i)= 1/sR.mu_f_nmes_p(i)
        sR.n_pst_mes(i) = sP.L_mes(i)/sP.L_pst(i);
        sR.delta_t_ref_mes(i) = sP.L_mes(i)/sP.V_ref(i);
        sR.delta_t_tg_mes(i) = sR.delta_t_ref_mes(i) - sP.delta_t_est_mes(i) - (sP.delta_t_sem_pst(i) + sR.delta_t_rmp(i) + sR.delta_t_crz(i) )*sR.n_pst_mes(i)

        sR.V_med(i) = sP.V_crz(i)/(1 + sP.V_crz(i)^2/(2*sP.a_rmp(i)*sP.L_pst(i)))
        sR.V_med_kph(i,fat)= sR.V_med(i)*3.6
        sR.ee(i)= sP.L_pst(i)/sR.VV_pst(i); 
        sR.ee_kpl(i,fat)= (sP.L_pst(i)/1000)/(sR.VV_pst(i)*1000); 
        sR.M_Cp_v(i,fat)= - (sP.I_compra(i)*sR.mu_pm(i) - sP.I_revenda(i)*sR.mu_f_nmes_p(i)*sR.mu_pm(i))*sP.f_compra(i);
        sR.M_Op_v(i,fat)= - (sP.L_mes(i)*sP.kappa_comb(i)*sP.rho_comb(i)*sP.f_oper(i))/(sR.ee(i));
        if (i==6) // o caso do táxi é especial
            sR.M_Op_v(i,fat) = -(2*5*sP.dias_mes  + 1.5*sP.L_mes(6)/1000);         
        end 
        if (i==7) // o caso do ônibus é especial
            sR.M_Op_v(i,fat) = -(4.5*2*sP.dias_mes );            
        end
        sR.M_Tg_v(i,fat)= sR.delta_t_tg_mes(i)*sP.f_tg(i); 
        sR.M_cf_v(i,fat)= sP.L_mes(i)*sP.f_cf(i); 
        sR.M_sg_v(i,fat)= sP.L_mes(i)*sP.f_sg(i); 
        sR.M_pt_v(i,fat)= sP.I_compra(i)*sP.f_pt(i); 
        sR.M_CPS_v(i,fat) = sR.M_cf_v(i,fat) + sR.M_pt_v(i,fat) + sR.M_sg_v(i,fat) ;
        sR.M_glb_f_v(i,fat) = sR.M_Cp_v(i,fat) + sR.M_Op_v(i,fat)
        sR.M_glb_ft_v(i,fat) = sR.M_glb_f_v(i,fat) + sR.M_Tg_v(i,fat)
        sR.M_glb_fts_v(i,fat) = sR.M_glb_f_v(i,fat) + sR.M_Tg_v(i,fat) + sR.M_CPS_v(i,fat)
        sR.cc(i,fat) = -sR.M_glb_f_v(i,fat)/sP.S_mes(i)
        if sR.cc(i,fat) > sP.f_cmpr
            sR.M_glb_f_v(i,fat) = min(0,sR.M_glb_f_v(i,fat));
            sR.M_glb_ft_v(i,fat) = 0;
            sR.M_glb_fts_v(i,fat) = 0;
        end
    end
end
//

// títulos do eixo Y, dependendo da métrica
if sP.metrica == 1 then
    tit_Y = '$M^{(v)}_{\textit{Cp}}\quad[R\$\textit{/mes}]$'    
    vet_graf = sR.M_Cp_v;
elseif sP.metrica == 2 then 
    tit_Y = '$M^{(v)}_{\textit{Op}}$\quad[R\$\textit{/mes]$'    
    vet_graf = sR.M_Op_v;
elseif sP.metrica == 3 then 
    tit_Y = '$M^{(v)}_{\textit{Tg}}$\quad[R\$\textit{/mes}]$'    
    vet_graf = sR.M_Tg_v;
elseif sP.metrica == 4 then 
    tit_Y = '$M^{(v)}_{\textit{CPS}}\quad[R\$\textit{/mes}]$'    
    vet_graf = sR.M_CPS_v;
elseif sP.metrica == 5 then 
    tit_Y = '$M^{(v)}_{\textit{glb,f}}$\quad[R\$\textit{/mes}]$'    
    vet_graf = sR.M_glb_f_v;
elseif sP.metrica == 6 then 
    tit_Y = '$M^{(v)}_{\textit{glb,ft}}$\quad[R\$\textit{/mes}]$'    
    vet_graf = sR.M_glb_ft_v;
elseif sP.metrica == 7 then 
    tit_Y = '$M^{(v)}_{\textit{glb,fts}}\quad[R\$\textit{/mes}]$'    
    vet_graf = sR.M_glb_fts_v;
elseif sP.metrica == 8 then 
    tit_Y = '$\mathfrak{e}$\quad[\textit{km/litro}]$'    
    vet_graf = sR.ee_kpl;
elseif sP.metrica == 9 then 
    tit_Y = '$\mathfrak{c}$'    
    vet_graf = sR.cc;
end

// título do eixo X, dependendo do estudo realizado
if sP.estudo==1 then
    tit_X = '$L_{\textit{dia}}\quad[km]$'
    sP.val_fat = sP.val_fat/sP.dias_mes/1000;
else
    tit_X = '$S_{\textit{mes}}\quad[\textit{mil }R\$]$'
    sP.val_fat = sP.val_fat/1000;
end

// plota o gráfico desejado
corgraf = ["blue", "red", "turquoise", "green", "gold", "midnight blue", "brown"]    
xlabel(tit_X,'fontsize',4);      
ylabel(tit_Y,'fontsize',4);
for i=1:sP.n_veic
    plot2d(sP.val_fat(:)',vet_graf(i,:),style=color(corgraf(i)))
    h = gca(); // get current axes
    h.children(1).children(1).thickness = 3;
    h.children(1).children(1).line_style = 1;   //1: contínua, 7: pontilhada, 8: tracejada
end



