
// FUNÇÃO DE CÁLCULO DOS COEFICIENTES DAS CURVAS DO MOTOR
function PM=eval_parmot(MOT)
    // criação da matriz A da correlação de Pmax
    AP = [1  MOT.w_motor_P_pico MOT.w_motor_P_pico^2 MOT.w_motor_P_pico^3;
    0 1 2*MOT.w_motor_P_pico 3*MOT.w_motor_P_pico^2;
    1  MOT.w_motor_corte_min MOT.w_motor_corte_min^2 MOT.w_motor_corte_min^3;
    -1/(MOT.w_motor_T_pico^2) 0 1 2*MOT.w_motor_T_pico]
    // criação do vetor b da correlação de Pmax
    bP = [MOT.P_pico; 0; MOT.P_corte_min; 0];
    // calculo dos coeficientes a`s de Pmax
    PM.a = AP\bP;
   
    // normalização da rotação
    MOT.w_motor_eta_otimo_norm = (MOT.w_motor_eta_otimo-MOT.w_motor_corte_min)/(MOT.w_motor_corte_max - MOT.w_motor_corte_min);
    // criação da matriz A da correlação de eta_w
    AW = [1  1  1;
    1 MOT.w_motor_eta_otimo_norm  MOT.w_motor_eta_otimo_norm^2   ;
    0 1 2*MOT.w_motor_eta_otimo_norm]
    // criação do vetor b da correlação de eta_w
    bW = [MOT.eta_rot_rot_max; 1; 0];
   // calculo dos coeficientes b`s de eta_w
    PM.b = AW\bW;

    // criação da matriz A da correlação de eta_a
    AA = [ 1 0  0;
    1 MOT.alpha_otimo MOT.alpha_otimo^2 ;
    0 1 2*MOT.alpha_otimo]
    // criação do vetor b da correlação de eta_a
    bA = [MOT.eta_alpha_alpha_min; 1; 0];
   // calculo dos coeficientes c`s de eta_a
    PM.c = AA\bA;
endfunction

// FUNÇÃO POTÊNCIA MÁXIMA DO MOTOR
function Pmax=eval_Pmotmax (MOT,PM,rot)
   if rot>MOT.w_motor_corte_max then  
       Pmax = 0;
   elseif rot<MOT.w_motor_corte_min then
       rotref = MOT.w_motor_corte_min;
       Pmax=(PM.a(1) + PM.a(2)*rotref + PM.a(3)*rotref^2 + PM.a(4)*rotref^3)*rot/MOT.w_motor_corte_min;
   else
       Pmax = PM.a(1) + PM.a(2)*rot + PM.a(3)*rot^2 + PM.a(4)*rot^3;
   end
endfunction

// FUNÇÃO EFICIÊNCIA DO MOTOR
function eficiencia=eval_efic (MOT,PM,rot,a)
    w_norm = (rot-MOT.w_motor_corte_min)/(MOT.w_motor_corte_max - MOT.w_motor_corte_min);
    eficiencia = MOT.eta_nominal*(PM.b(1) + PM.b(2)*w_norm + PM.b(3)*w_norm^2)*(PM.c(1) + PM.c(2)*a + PM.c(3)*a^2);
endfunction

// FUNÇÃO POTÊNCIA DO COMBUSTÍVEL
function Pcomb=eval_Pcomb (MOT,PM,rot,a)
    P_motor = eval_Pmotmax(MOT,PM,rot);   // considerando a embreagem para reduzir a potência
    if rot>MOT.w_motor_corte_max 
           rot = MOT.w_motor_corte_max;
    elseif rot<MOT.w_motor_corte_min then
        rot=MOT.w_motor_corte_min
    end
    eficiencia = eval_efic(MOT,PM,rot,a);  // mas o motor está operando na rotação mínima
    Pcomb = (a*P_motor + MOT.P_int)/eficiencia;
endfunction


// FUNÇÃO POTÊNCIA MÁXIMA NA RODA
function [Proda,marcha_esc,rotacao]=eval_Pmaxroda(MOT,PM,TRN,V)
    // número de marchas
    n_marchas = size(TRN.csi_marcha_i,2);  
    // calcula para todas as marchas
    for marcha = 1:n_marchas  
        // rotação do motor
        rot(marcha) = TRN.csi_prim*TRN.csi_marcha_i(marcha)*TRN.csi_fin*V/TRN.RRoda   
        Pmot(marcha)=eval_Pmotmax(MOT,PM,rot(marcha));
    end
    // escolhe a máxima potência entre todas as marchas
    [Pmax,marcha_esc] = max(Pmot(:));
    if  rot>MOT.w_motor_corte_max  then
        marcha_esc = n_marchas  
    elseif rot<MOT.w_motor_corte_min then
        marcha_esc = 1
    end
    rotacao = rot(marcha_esc);
    // potência na roda é diminuida pela eficiência da transmissão
    Proda = Pmax*TRN.eta_t;
endfunction











