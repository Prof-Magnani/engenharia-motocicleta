//------------------------------------------------------------
// inicializaçao e parâmetros computacionais
clc;
clear;
getd('.\');
[sP,sA] = get_parametros_v2();                          // lê dados e calcula variáveis auxiliares
// parâmetros físicos
sR.rot = (1500:500:9000)*(2*sP.pi/60);                  // vetor de rotações estudadas
sR.red_eta_vol = 1.0                                    // redutor de eficiência volumétrica (caso estudado)
// parâmetros numéricos
sR.NP = 960;                                            // múltiplo de 4 (tempos)
sR.NP = sR.NP - pmodulo(sR.NP,4);                       // garante que será múltiplo de 4 (tempos)
sR.NT = size(sR.rot,2);                                 // número de rotações estudadas
sR.dteta = (sP.teta_fim-sP.teta_ini)/sR.NP;

// criação do vetor de ângulo e cálculo do volume
for i=1:sR.NP
    sV.teta(i) = (i-1)/(sR.NP-1)*(sP.teta_fim-sP.teta_ini) + sP.teta_ini;   // vetor de ângulos do virabrequim estudados
    [sV.V(i),sV.A_troca(i)] = get_vol(sP,sA,sV.teta(i));                    // volume e área de troca para cada ângulo do virabrequim
end

// loop do tempo
for it=1:sR.NT
    // rotação, condições gerais do ciclo e eficiência volumétrica
    sR.dt = (4*sP.pi/sR.rot(it))/sR.NP;                                                                // dt correspondente ao dteta
    sR.w_linha = (sR.rot(it)-sP.w_motor_corte_min)/(sP.w_motor_corte_max-sP.w_motor_corte_min);        // rotação normalizada
    sR.eta_vol_max(it) = (sA.PMd(1) + sA.PMd(2)*sR.w_linha + sA.PMd(3)*sR.w_linha^2);                  // eficiência volumétrica máxima
    sR.eta_vol(it) = sR.red_eta_vol*sR.eta_vol_max(it);                                                // eficiência volumétrica estudada
    sR.p_a = sP.p_amb*sR.eta_vol(it);                                                                  // pressão da aspiração/descarga
    sR.n_res = sR.p_a*sA.V_PMS/(sP.R_bar*sP.T_a);                                                      // número de moles na mistura residual

    // cálculo da composição
    for i=1:sR.NP
        sV.xi(i) = get_xi(sP,sV.teta(i),sR.rot(it));                                                   // progresso da queima em função de teta
        [sV.comp(i,:),sV.nu_soma(i),sV.uf_carga(i),sV.uf_est_carga(i),sV.cv_carga(i),sV.cp_carga(i),sV.M(i)] = get_comp(sP,sV.teta(i),sV.V(i),sV.xi(i));    // composição da mistura em função de teta
    end

    // aspiração    
    for i=1:sR.NP*1/4
        sV.p(i) = sR.p_a;                                                                    // pressão da carga
        sV.T(i) = sP.T_a;                                                                    // temperatura da carga
        sV.n_carga(i) = sV.p(i)*sV.V(i)/(sP.R_bar*sV.T(i))                                   // número de moles da carga
        sV.m_carga(i) = sV.n_carga(i)*sV.M(i);                                               // massa da carga
        sV.W_dot_inst(i) = 0;                                                                // potência no pistão
        sV.Q_dot_inst(i) = 0;                                                                // troca de calor
        sV.U(i) = TparaU(sP,sV.uf_est_carga(i),sV.cv_carga(i),sV.n_carga(i),sV.T(i));        // interna energia da carga
    end

    //sR.n_comb = (sV.n_carga(sR.NP*1/4) - sR.n_res)*sV.comp(sR.NP*1/4,1);  // número de moles de combustível ao final da aspiração
    sR.n_comb = sV.n_carga(sR.NP*1/4)*sV.comp(sR.NP*1/4,1);  // número de moles de combustível ao final da aspiração

    // compressão
    for i=sR.NP*1/4+1:sR.NP*2/4
        sV.m_carga(i) = sV.m_carga(i-1);                                                // massa da carga
        sV.n_carga(i) = sV.m_carga(i)/sV.M(i);                                          // número de moles da carga
        sV.W_dot_inst(i) = sV.p(i-1)*(sV.V(i)-sV.V(i-1))/sP.eta_comp/sR.dt;             // potência no pistão
        sV.Q_dot_inst(i) = sP.U_refri*sV.A_troca(i)*(sP.T_refri - sV.T(i-1));           // troca de calor
        sV.U(i) = sV.U(i-1) + sV.Q_dot_inst(i)*sR.dt - sV.W_dot_inst(i)*sR.dt;          // variação da interna energia da carga
        sV.T(i) = UparaT(sP,sV.uf_est_carga(i),sV.cv_carga(i),sV.n_carga(i),sV.U(i));   // temperatura da carga
        sV.p(i) = sV.n_carga(i)*sP.R_bar*sV.T(i)/sV.V(i);                               // pressão da carga
    end

    // expansão
    for i=sR.NP*2/4+1:sR.NP*3/4
        sV.m_carga(i) = sV.m_carga(i-1);                                                // massa da carga
        sV.n_carga(i) = sV.m_carga(i)/sV.M(i);                                          // número de moles da carga
        sV.W_dot_inst(i) = sV.p(i-1)*(sV.V(i)-sV.V(i-1))*sP.eta_exp/sR.dt;              // potência no pistão
        sV.Q_dot_inst(i) = sP.U_refri*sV.A_troca(i)*(sP.T_refri - sV.T(i-1));           // troca de calor
        sV.U(i) = sV.U(i-1) + sV.Q_dot_inst(i)*sR.dt - sV.W_dot_inst(i)*sR.dt;          // variação da interna energia da carga
        sV.T(i) = UparaT(sP,sV.uf_est_carga(i),sV.cv_carga(i),sV.n_carga(i),sV.U(i));   // temperatura da carga
        sV.p(i) = sV.n_carga(i)*sP.R_bar*sV.T(i)/sV.V(i);                               // pressão da carga
    end

    // descarga
    for i=sR.NP*3/4+1:sR.NP*4/4
        sV.p(i) = sR.p_a;                                                                    // pressão da carga
        sV.T(i) = sV.T(i-1);                                                                 // temperatura da carga
        sV.n_carga(i) = sV.p(i)*sV.V(i)/(sP.R_bar*sV.T(i-1))                                 // número de moles da carga
        sV.m_carga(i) = sV.n_carga(i)*sV.M(i);                                               // massa da carga
        sV.W_dot_inst(i) = 0;                                                                // potência no pistão
        sV.Q_dot_inst(i) = 0;                                                                // troca de calor
        sV.U(i) = TparaU(sP,sV.uf_est_carga(i),sV.cv_carga(i),sV.n_carga(i),sV.T(i));        // energia da carga
    end

    // INTEGRAIS
    sR.Q_dot_ciclo(it) = sum(sV.Q_dot_inst*sR.dteta)/(4*sP.pi);                             // troca de calor no ciclo, W
    sR.P_pist(it) = sum(sV.W_dot_inst*sR.dteta)/(4*sP.pi);                                  // potência do pistão no ciclo, W
    sR.E_dot_comb(it) =  sR.n_comb*sP.M(1)*sA.PC_comb_massa*sR.rot(it)/(4*sP.pi);           // taxa de entrada de energia de combustível   no ciclo, W
    sR.eta_motor(it) = sR.P_pist(it)/sR.E_dot_comb(it);                                     // eficiência do motor
    sR.T_residual(it) = sV.T(sR.NP*4/4);                                                    // temperatura residual, K

    // RESULTADOS DERIVADOS DAS INTEGRAIS
    sR.T_pist(it) = sR.P_pist(it)/sR.rot(it);                                                   // torque do pistão, N.m
    sR.P_pist_CV(it) = sR.P_pist(it)/736;                                                       // potência do  pistão, CV
    sR.T_pist_kgfm(it) = sR.T_pist(it)/sP.g;                                                    // torque do pistão, kgf.m
    sR.P_int(it) = sR.rot(it)*(sR.w_linha*(sP.T_int_w_max - sP.T_int_w_min) + sP.T_int_w_min);  // potência interna, W
    sR.P_motor(it) = sR.P_pist(it) - sR.P_int(it);                                              // potência do motor, W
    sR.eta_vbq(it) = sR.P_motor(it)/sR.E_dot_comb(it);                                          // eficiência até o virabrequim
    sR.T_motor(it) = sR.P_motor(it)/sR.rot(it);                                                 // torque do motor, N.m
    sR.P_motor_CV(it) = sR.P_motor(it)/736;                                                     // potência do motor, CV
    sR.T_motor_kgfm(it) = sR.T_motor(it)/sP.g;                                                  // torque do motor, kgf.m
end


// TEMPLATES DE GRÁFICOS
//------------------------------------------------------------
//plot (sV.teta*180/sP.pi,sV.V*1e6,'k','LineWidth',3)        
//xlabel('$\theta\quad[^{o}]$','fontsize',4);       
//ylabel('$V_{\text{cil}}\quad[cc]$','fontsize',4);    
//h = gca(); // get current axes
//h.data_bounds = [0, 0 ; 720 , 350];
//h.tight_limits=["on","off"];

//plot (sV.teta*180/sP.pi,sV.xi,'k','LineWidth',3)        
//xlabel('$\theta\quad[^{o}]$','fontsize',4);       
//ylabel('$\xi$','fontsize',4);    
//h = gca(); // get current axes
//h.data_bounds = [0, 0 ; 720 , 1];
//h.tight_limits=["on","off"];

//plot (sV.teta*180/sP.pi,sV.comp(:,1),'k','LineWidth',3)        
//plot (sV.teta*180/sP.pi,sV.comp(:,2),'k','LineWidth',3)        
//plot (sV.teta*180/sP.pi,sV.comp(:,3),'k','LineWidth',3)        
//plot (sV.teta*180/sP.pi,sV.comp(:,4),'k','LineWidth',3)        
//plot (sV.teta*180/sP.pi,sV.comp(:,5),'k','LineWidth',3)        
//xlabel('$\theta\quad[^{o}]$','fontsize',4);       
//ylabel('$y_{i}$','fontsize',4);    
//h = gca(); // get current axes
//h.data_bounds = [0, 0 ; 720 , 1];
//h.tight_limits=["on","off"];
//
//plot (sV.teta*180/sP.pi,sV.m_carga,'k','LineWidth',3)        
//xlabel('$\theta\quad[^{o}]$','fontsize',4);       
//ylabel('$m_{\textit{carga}}\quad[\textit{kg}]$','fontsize',4);    
//h = gca(); // get current axes
//h.data_bounds = [0, 0 ; 720, 0.0004];
//h.tight_limits=["on","off"];

//plot (sV.teta*180/sP.pi,sV.n_carga,'k','LineWidth',3)        
//xlabel('$\theta\quad[^{o}]$','fontsize',4);       
//ylabel('$n_{\textit{carga}}\quad[\textit{kmol}]$','fontsize',4);    
//h = gca(); // get current axes
//h.data_bounds = [0, 0 ; 720, 0.000014];
//h.tight_limits=["on","on"];

//plot (sV.teta*180/sP.pi,sV.p/1000,'b','LineWidth',3)        
//plot (sV.teta*180/sP.pi,(sV.T-273),'r','LineWidth',3)        
//xlabel('$\theta\quad[^{o}]$','fontsize',4);       
//ylabel('$p{}[\textit{kPa}],\quad T{}[\textit{C}]$','fontsize',4);    
//h = gca(); // get current axes
//h.data_bounds = [0, 0 ; 720, 8500];
//h.tight_limits=["on","on"];

//plot (sV.teta*180/sP.pi,sV.p/1000,'k','LineWidth',3)        
//xlabel('$\theta\quad[^{o}]$','fontsize',4);       
//ylabel('$p\quad[kPa]$','fontsize',4);    
//h = gca();  ;;get current axes
//h.data_bounds = [240, 0 ; 540, 8500];
//h.tight_limits=["on","on"];

//plot (sV.teta*180/sP.pi,(sV.T-273),'k','LineWidth',3)        
//xlabel('$\theta\quad[^{o}]$','fontsize',4);       
//ylabel('$T{}[\textit{C}]$','fontsize',4);    
//h = gca(); // get current axes
//h.data_bounds = [0, 0 ; 720, 4000];
//h.tight_limits=["on","on"];

//plot (sR.rot/(2*sP.pi/60),sR.P_motor_CV','k','LineWidth',2)
//xlabel('$\omega\quad[rpm]$','fontsize',4);       
//ylabel('$P_{\text{motor}}\quad[CV]$','fontsize',4); 
//h = gca(); // get current axes
//h.data_bounds = [1500, 0 ; 9000, 40];
//h.tight_limits=["on","on"];

//plot (sR.rot/(2*sP.pi/60),sR.P_motor_CV','k','LineWidth',3)
//plot (sR.rot/(2*sP.pi/60),sR.T_motor','k','LineWidth',3)                
//xlabel('$\omega\quad[rpm]$','fontsize',4);       
//ylabel('$P_{\text{motor}}{}[CV],\quad T_{\text{motor}}{}[N.m]$','fontsize',4);    

//plot (sR.rot/(2*sP.pi/60),sR.T_motor','k','LineWidth',4)                
//xlabel('$\omega\quad[rpm]$','fontsize',4);       
//ylabel('$T_{\text{motor}}\quad[N.m]$','fontsize',4);    

//plot (sV.teta*180/sP.pi,sV.W_dot_inst,'k','LineWidth',3)        
//plot (sV.teta*180/sP.pi,sV.Q_dot_inst,'k','LineWidth',1)        
//xlabel('$\theta\quad[^{o}]$','fontsize',4);       
//ylabel('$\dot{W}_{\textit{inst}}, \dot{Q}_{\textit{inst}}\quad[W] $','fontsize',4);    
//h = gca(); // get current axes
//h.data_bounds = [0, -300000 ; 720, 400000];
//h.tight_limits=["on","on"]; 

//plot (sR.rot/(2*sP.pi/60),sR.eta_motor','k','LineWidth',3)        
//xlabel('$\omega\quad[rpm]$','fontsize',4);       
//ylabel('$\eta_{\text{m}}$','fontsize',4);   
//------------------------------------------------------------
