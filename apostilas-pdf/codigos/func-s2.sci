//------------------------------------------------------------
// COEFICIENTES DAS CURVAS DO MOTOR
function [PMa,PMb,PMc,PMd] = eval_parmot_v2(sP)
    // criação da matriz A da correlação de Pmax
    AP = [1  sP.w_motor_P_pico sP.w_motor_P_pico^2 sP.w_motor_P_pico^3;
    0 1 2*sP.w_motor_P_pico 3*sP.w_motor_P_pico^2;
    1  sP.w_motor_corte_min sP.w_motor_corte_min^2 sP.w_motor_corte_min^3;
    -1/(sP.w_motor_T_pico^2) 0 1 2*sP.w_motor_T_pico]
    // criação do vetor b da correlação de Pmax
    bP = [sP.P_pico; 0; sP.P_corte_min; 0];
    // calculo dos coeficientes a`s de Pmax
    PMa = AP\bP;
   
    // normalização da rotação
    sP.w_motor_eta_otimo_norm = (sP.w_motor_eta_otimo-sP.w_motor_corte_min)/(sP.w_motor_corte_max - sP.w_motor_corte_min);
    // criação da matriz A da correlação de eta_w
    AW = [1  1  1;
    1 sP.w_motor_eta_otimo_norm  sP.w_motor_eta_otimo_norm^2   ;
    0 1 2*sP.w_motor_eta_otimo_norm]
    // criação do vetor b da correlação de eta_w
    bW = [sP.eta_rot_rot_max; 1; 0];
   // calculo dos coeficientes b`s de eta_w
    PMb = AW\bW;

    // criação da matriz A da correlação de eta_a
    AA = [ 1 0  0;
    1 sP.alpha_otimo sP.alpha_otimo^2 ;
    0 1 2*sP.alpha_otimo]
    // criação do vetor b da correlação de eta_a
    bA = [sP.eta_alpha_alpha_min; 1; 0];
   // calculo dos coeficientes c`s de eta_a
    PMc = AA\bA;

    // acrescentado na versão 2
    
    // normalização da rotação
    sP.w_motor_eta_vol_otimo_norm = (sP.w_motor_eta_vol_otimo-sP.w_motor_corte_min)/(sP.w_motor_corte_max - sP.w_motor_corte_min);
    // criação da matriz AV da correlação de eta_vol
    AV = [ 1 0  0;
    1 sP.w_motor_eta_vol_otimo_norm sP.w_motor_eta_vol_otimo_norm^2 ;
    0 1 2*sP.w_motor_eta_vol_otimo_norm]
    // criação do vetor b da correlação de eta_vol
    bV = [sP.eta_vol_rot_min; sP.eta_vol_max; 0];
   // calculo dos coeficientes c`s de eta_a
    PMd = AV\bV;
endfunction
//------------------------------------------------------------

//------------------------------------------------------------
// FUNÇÃO POTÊNCIA MÁXIMA DO MOTOR EM FUNÇÃO DA ROTAÇÃO
function Pmax=eval_Pmotmax (sP,sA,rot)
   if rot>sP.w_motor_corte_max then  
       Pmax = 0;
   elseif rot<sP.w_motor_corte_min then
       rotref = sP.w_motor_corte_min;
       Pmax=(sA.PMa(1) + sA.PMa(2)*rotref + sA.PMa(3)*rotref^2 + sA.PMa(4)*rotref^3)*rot/sP.w_motor_corte_min;
   else
       Pmax = sA.PMa(1) + sA.PMa(2)*rot + sA.PMa(3)*rot^2 + sA.PMa(4)*rot^3;
   end
endfunction
//------------------------------------------------------------

//------------------------------------------------------------
// FUNÇÃO EFICIÊNCIA DO MOTOR EM FUNÇÃO DA ROTAÇÃO E DE ALPHA
function eficiencia=eval_efic (sP,sA,rot,a)
    w_norm = (rot-sP.w_motor_corte_min)/(sP.w_motor_corte_max - sP.w_motor_corte_min);
    eficiencia = sP.eta_nominal*(sA.PMb(1) + sA.PMb(2)*w_norm + sA.PMb(3)*w_norm^2)*(sA.PMc(1) + sA.PMc(2)*a + sA.PMc(3)*a^2);
endfunction
//------------------------------------------------------------

//------------------------------------------------------------
// POTÊNCIA DO COMBUSTÍVEL EM FUNÇÃO DA ROTAÇÃO E DE ALPHA
function Pcomb=eval_Pcomb (sP,sA,P_int,rot,a)
    P_motor = eval_Pmotmax(sP,sA,rot);   // considerando a embreagem para reduzir a potência
    if rot>sP.w_motor_corte_max 
           rot = sP.w_motor_corte_max;
    elseif rot<sP.w_motor_corte_min then
        rot=sP.w_motor_corte_min
    end
    eficiencia = eval_efic(sP,sA,rot,a);  // mas o motor está operando na rotação mínima
    Pcomb = (a*P_motor + P_int)/eficiencia;
endfunction
//------------------------------------------------------------

//------------------------------------------------------------
// POTÊNCIA MÁXIMA NA RODA EM FUNÇÃO DA VELOCIDADE
function [Proda,marcha_esc,rotacao]=eval_Pmaxroda(sP,sA,V)
    // número de marchas
    n_marchas = size(sP.csi_marcha_i,2);  
    // calcula para todas as marchas
    for marcha = 1:n_marchas  
        // rotação do motor
        rot(marcha) = sP.csi_prim*sP.csi_marcha_i(marcha)*sP.csi_fin*V/sP.RRoda   
        Pmot(marcha)=eval_Pmotmax(sP,sA,rot(marcha));
    end
    // escolhe a máxima potência entre todas as marchas
    [Pmax,marcha_esc] = max(Pmot(:));
    if  rot>sP.w_motor_corte_max  then
        marcha_esc = n_marchas  
    elseif rot<sP.w_motor_corte_min then
        marcha_esc = 1
    end
    rotacao = rot(marcha_esc);
    // potência na roda é diminuida pela eficiência da transmissão
    Proda = Pmax*sA.eta_t;
endfunction
//------------------------------------------------------------

//------------------------------------------------------------
// VOLUME DO CICLINDRO EM FUNÇÃO DO ANGULO DO VIRABREQUIM
function [V,A_troca] = get_vol(sP,sA,teta)
    phi = asin(sA.L_M/sA.L_B*sin(teta));                        // ângulo da biela com a vertical
    x_pist = sA.L_B*cos(phi) + sA.L_M*cos(teta);                // posição do pistão em relação ao eixo do virabrequim
    h_cil = sA.h_vbq - x_pist;      // altura do cilindro
    V = sA.A_pist*h_cil;                                        // volume do cilindro
    A_troca = sA.A_cab  + sA.A_pist + h_cil*sP.pi*sA.D_pist;    // área de troca do cilindro
endfunction
//------------------------------------------------------------

//------------------------------------------------------------
// PROGRESSO DA QUEIMA EM FUNÇÃO DO ÂNGULO DO VIRABREQUIM
function xi = get_xi(sP,teta,w)
    sP.delta_teta_delay = sP.delta_teta_delay_wref*w/sP.w_ref;                              // calculo do intervalo de ângulo de delay em função da rotação
    if teta < (sP.teta_qp - sP.delta_teta_delay) then               
        xi = 0;                                                                             // antes da compressão não  há reação química
    elseif teta < sP.teta_qp then                                                           // queima do delay
        xi = (teta - (sP.teta_qp - sP.delta_teta_delay))/sP.delta_teta_delay*sP.f_delay;
    elseif teta < sP.teta_qp + sP.delta_teta_queima then
        xi = (teta - sP.teta_qp)/sP.delta_teta_queima*(1-sP.f_delay) + sP.f_delay;          // queima principal
    else    
        xi = 1;                                                                             // no final só há produtos
    end
endfunction
//------------------------------------------------------------

//------------------------------------------------------------
// COMPOSIÇÃO EM FUNÇÃO DO ÂNGULO DO VIRABREQUIM
function [y,nu_soma,uf,uf_est,cv,cp,M] = get_comp(sP,teta,V,xi)
    nu_soma = 0;
    // durante a aspiração, há uma mistura dos gases residuais com os reagentes aspirados
    if (teta<sP.teta_a) then
        frac_res_inst = sA.V_PMS/V;
        for i=1:sP.n_comp
            nu_i_carga(i) =  sA.nu(1,i)*(1-frac_res_inst)  +  sA.nu(2,i)*frac_res_inst
            nu_soma = nu_soma + nu_i_carga(i);
        end
    // após a aspiração, a composição é dada pelo progresso da queima
    else
      for i=1:sP.n_comp
          nu_i_carga(i) =  1/sP.r_v*((1-xi)*(sP.r_v-1)*sA.nu(1,i)  + (xi*(sP.r_v-1)+1)*sA.nu(2,i));
          nu_soma = nu_soma + nu_i_carga(i);
      end
    end
    
    // a fração molar é o coeficiente estequiométrico  da substância dividido pela soma dos coeficientes estequiométricos de todas as substâncias
    y = nu_i_carga/nu_soma;
    
    // com a fração molar, as outras propriedades podem ser calculadas como médias ponderadas
    uf = 0;
    cv = 0;
    M = 0;
    for i=1:sP.n_comp
        uf = uf + y(i)*sP.uf(i);
        cv = cv + y(i)*sP.cv(i);
        M = M +  y(i)*sP.M(i) ;
    end
    uf_est = uf + cv*sP.T0;   
    cp = cv + sP.R_bar;
   
endfunction
//------------------------------------------------------------

//------------------------------------------------------------
// ENERGIA  INTERNA  EM FUNÇÃO DA TEMPERATURA
function U = TparaU(sP,uf_est,cv,n_carga,T)
   U = n_carga*(uf_est + cv*T);
endfunction
//------------------------------------------------------------

//------------------------------------------------------------
// TEMPERATURA EM FUNÇÃO DA ENERGIA INTERNA
function T = UparaT(sP,uf_est,cv,n_carga,U)
    T = (U - n_carga*uf_est)/(n_carga*cv);
endfunction
//------------------------------------------------------------







