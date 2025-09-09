//------------------------------------------------------------
//// COEFICIENTES DAS CURVAS DO MOTOR
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

// ------------------------------------------------------/
// RETORNA A VELOCIDADsE, ACELERAÇÃO (lado direito da EDO) E VARIÁVEIS AUXILIARES
function [f,sE]=S(t,Q)
    //extracao das variaveis
    x = Q(1,1);          // x                   
    V = Q(2,1);          // V

    // modelo do motor e da pista
    [sE.Pmaxroda,sE.mar,sE.rot]=eval_Pmaxroda(sP,sA,V)  // calcula a Pmaxroda
    sE = get_pista_Fpos(sE,x)                    // pega informações sobre a pista e calcula forças que dependem apenas da posição

    // piloto controla a velocidade desejada via acelerador e freio
    sE =  get_alphabeta(sE,x,V);   // calcula alpha e beta
    sE.a = get_aceleracao(sE,V);                                             // calcula aceleração
    sE = get_uti(sE,V);                                                      // calcula utilizações                     

    // piloto controle a utilização via acelerador
    while (sE.u_ad_t>sP.u_alpha_max | sE.u_ad_d>sP.u_alpha_max | sE.u_no_d>sP.u_alpha_max) & sE.alpha>0
        sE.alpha = sE.alpha - sP.delta_alpha;
        sE.a = get_aceleracao(sE,V);
        sE = get_uti(sE,V);           
    end
    sE.alpha = min(max(sE.alpha,0),1);        // limita alpha entre 0 e 1;
 
    // piloto controle a utilização via freio
    while (sE.u_ad_t>sP.u_beta_max | sE.u_ad_d>sP.u_beta_max| sE.u_no_t>sP.u_beta_max) & (sE.beta_t>0 & sE.beta_d>0)
        sE.beta_t = max(sE.beta_t - sP.delta_beta,0);
        sE.beta_d = max(sE.beta_d - sP.delta_beta,0);
        sE.a = get_aceleracao(sE,V);
        sE = get_uti(sE,V);             
    end
     sE.beta_t = min(max(sE.beta_t,0),1);     // limita beta_t entre 0 e 1
     sE.beta_d = min(max(sE.beta_d,0),1);     // limita beta_d entre 0 e 1
      sE.a = get_aceleracao(sE,V); 
    // potência interna, potência de combustível e eficiência
    w_linha = (sE.rot-sP.w_motor_corte_min)/(sP.w_motor_corte_max-sP.w_motor_corte_min); // rotação normalizada
    sE.Pint = sE.rot*(w_linha*(sP.T_int_w_max - sP.T_int_w_min) + sP.T_int_w_min);        // potência dissipada interna
    sE.Pcomb=eval_Pcomb(sP,sA,sE.Pint,sE.rot,sE.alpha)                                      // Pcomb
    sE.eta = eval_efic (sP,sA,sE.rot,sE.alpha);                                            // eficiência do motor

    // retorna o lado direito  das equações diferenciais
    f(1)= V;              
    f(2)= sE.a;
endfunction
// ------------------------------------------------------/

// ------------------------------------------------------/
// CALCULA PISTA E FORÇAS QUE DEPENDEM APENAS DA POSIÇÃO
function sE=get_pista_Fpos(sE,x)
    
    // modelo da pista
    sE.teta = get_teta_pista(x);            // angulo de aclive
    sE.mu = get_coefmu(x);                // coeficiente de atrito
    sE.W = get_W(x);                  // velocidade do vento [m/s]
    sE.RCurvatura = get_RCurvatura(x);
 
    // forças que dependem da posição
    sE.Fgrx = -sA.m_c*sP.g*sin(sE.teta);   // componente x da gravidade
    sE.Fgry = -sA.m_c*sP.g*cos(sE.teta);   // componente y da gravidade
    sE.Faer = -sP.k_A*(V-sE.W)^2;            // arrasto aerodinamio
    sE.Frol = -sP.C_R*abs(sE.Fgry);        // resistencia a rolagem    
endfunction
// ------------------------------------------------------/

// ------------------------------------------------------/
// RETORNA ACLIVE DA PISTA
function [teta_pista] = get_teta_pista(x)
    teta_pista = sP.teta_pista;
endfunction
// ------------------------------------------------------/

// ------------------------------------------------------/
// RETORNA VENTO NA PISTA
function [W] = get_W(x)
    W = sP.W;
endfunction
// ------------------------------------------------------/

// ------------------------------------------------------/
// RETORNA ATRITO DA PISTA
function [mu] = get_coefmu(x)
    mu = sP.mu;
endfunction
// ------------------------------------------------------/

// ------------------------------------------------------/
// RETORNA RAIO DE CURVATURA DA PISTA
function [raio] = get_RCurvatura(x)
    if x< sP.L_tre1 then
      raio = sP.R_tre1;
    elseif x< sP.L_tre1+sP.L_tre2 then
       raio = sP.R_tre2; 
    else
      raio = sP.R_tre3;
    end
endfunction
// ------------------------------------------------------/

// ------------------------------------------------------/
// RETORNA ACELERAÇÃO
function [a] =  get_aceleracao(sE,V)
   a = (sE.alpha*sE.Pmaxroda/V+sE.Faer+sE.Frol+sE.Fgrx+sE.beta_t*sE.mu*sE.Fgry + (sE.beta_d-sE.beta_t)*sE.mu/sP.p_d*( -sE.Faer*sP.h_CP + -sE.Fgrx*sP.h_CG + sE.Fgry*sP.p_CG)  )/(sA.m_c*(1-(sE.beta_d-sE.beta_t)*sE.mu*sP.h_CG/sP.p_d))
endfunction
// ------------------------------------------------------/

// ------------------------------------------------------/
// RETORNA FORÇAS E UTILIZACOES
function [sE]=get_uti(sE,V)
 
    get_pista_Fpos(x)                                  // obtem informações sobre a pista
    
    // calculo das forcas
    sE.Fine = -sE.a*sA.m_c;                                   // forca de inercia
    sE.N_d = (sE.Fine*sP.h_CG + sE.Faer*sP.h_CP + sE.Fgrx*sP.h_CG - sE.Fgry*sP.p_CG)/sP.p_d;  // normal dianteira
    sE.N_t = -sE.N_d - sE.Fgry;                                // normal traseira
    sE.Ffrn_t = -sE.beta_t*sE.mu*sE.N_t;                        // força de frenagem traseira
    sE.Ffrn_d = -sE.beta_d*sE.mu*sE.N_d;                        // força de frenagem dianteira
    sE.Fpro = sE.alpha*sE.Pmaxroda/V;                          // força de propulsão
    sE.Fcen_d = sA.m_c/sE.RCurvatura*(V^2)*(sP.p_CG/sP.p_d)   // força centrípeta dianteira
    sE.Fcen_t = sA.m_c/sE.RCurvatura*(V^2)*(1-sP.p_CG/sP.p_d) // força centrípeta traseira
    sE.Fat_disp_d = sE.mu*sE.N_d;                              // força de atrito disponível na dianteira
    sE.Fat_disp_t = sE.mu*sE.N_t;                              // força de atrito disponível na traseira
    sE.Fat_d = sqrt(sE.Ffrn_d^2 + sE.Fcen_d^2)                 // força de atrito na dianteira
    sE.Fat_t = sqrt((sE.Fpro+sE.Ffrn_t)^2 + sE.Fcen_t^2)        // força de atrito na traseira
    sE.Nest_d = - sE.Fgry*(sP.p_CG/sP.p_d);                   // normal dianteira com a moto estática
    sE.Nest_t = -sE.Fgry*(1-sP.p_CG/sP.p_d)                   // normal traseira com a moto estática
 
    // calculo das utilizações
    sE.u_ad_d = abs(sE.Fat_d)/abs(sE.Fat_disp_d);              // utilização da aderência dianteira
    sE.u_ad_t = abs(sE.Fat_t)/abs(sE.Fat_disp_t);              // utilização da aderência traseira
    sE.u_no_d = 1 - sE.N_d/sE.Nest_d;                          // utilização da normal dianteira
    sE.u_no_t = 1 - sE.N_t/sE.Nest_t;                          // utilização da normal traseira
    
endfunction
// ------------------------------------------------------/

// ------------------------------------------------------/
// RETORNA ACELERADOR E FREIO
function sE =  get_alphabeta(sE,x,V)
    
    // calculo da aceleração máxima
    sE.alpha = 1;
    sE.beta_t = 0;
    sE.beta_d = 0;
    sE.a_max = get_aceleracao(sE,V);        // acelearação com uso máximo de acelerador
    sE.a_max = max(sE.a_max,0);

    // calculo da aceleração mínima
    sE.alpha = 0;
    sE.beta_t = 1;
    sE.beta_d = 1;
    sE.a_min= get_aceleracao(sE,V);       // aceleração com uso máximo de freio
    sE.a_min = min(sE.a_min,0);

    // calculo de alpha e beta buscando a velocidade do próximo passo
    dt = sP.NumPq;
    dx = V*dt;
    V_prox = min(get_Vseg(vet_Vseg,x+dx),get_Vdes(x+dx))         // mínimo entre velocidade de segurança e velocidade desejada
    a_des = (V_prox - V)/dt;                                    // aceleração desejada
    sE.alpha = min(max((a_des/sE.a_max),0),1);                  // limita alpha entre 0 e 1
    sE.beta_d = min(max((a_des/sE.a_min),0),1);                 // limita beta_d entre 0 e 1
    sE.beta_t = sE.beta_d;                                      // dois freios são acionados na mesma proporção
endfunction
// ------------------------------------------------------/

// ------------------------------------------------------/
// RETORNA VELOCIDADE DESEJADA
function [Vdes] = get_Vdes(x)
    Vdes = sP.V_cruz_des
    if x< sP.L_Vvar_des then
      Vdes = x/sP.L_Vvar_des*(sP.V_cruz_des-sP.V_ini_des) + sP.V_ini_des;
    else
      Vdes= sP.V_cruz_des
    end
endfunction
// ------------------------------------------------------/

// ------------------------------------------------------/
// CRIA VETOR DE VELOCIDADE DE SEGURANÇA
function [vet_Vseg] = get_vetor_Vseg()
    for i=1:sP.npVseg
        x_loc = sP.Lpista*(i-1)/(sP.npVseg-1);
        vet_Vseg(i) = sqrt(get_coefmu(x_loc)*sP.g*get_RCurvatura(x_loc))
        for j = i:sP.npVseg
           x_fut=sP.Lpista*(j-1)/(sP.npVseg-1);
           if (x_fut-x_loc<sP.dist_visao) then 
               vet_Vseg(i) = min(vet_Vseg(i),sqrt(get_coefmu(x_fut)*sP.g*get_RCurvatura(x_fut) + 2*sP.g*get_coefmu(x_fut)*(x_fut-x_loc)))
           end
        end
  end
  vet_Vseg = vet_Vseg*sP.f_red_Vseg
endfunction
// ------------------------------------------------------/
        
// ------------------------------------------------------/
// INTERPOLA A VELOCIDADE DE SEGURANÇA A PARTIR DO VETOR
function Vseg = get_Vseg(vet_Vseg,x)
    p = max(min(x/sP.Lpista,1),0)
    i = floor(p*(sP.npVseg-2))+1;
    d = p*(sP.npVseg-2) - floor(p*(sP.npVseg-2));
    Vseg = (1-d)*vet_Vseg(i) + d*vet_Vseg(i+1);
endfunction
// ------------------------------------------------------/


