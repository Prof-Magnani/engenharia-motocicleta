// comandos iniciais ao sistema computacional
clc;            // limpa o console
clear;          // limpa as variaveis
clearglobal;    // limpa as variaveis globais


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
    bA = [MOT.eta_alpha_rot_min; 1; 0];
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
        Tmot(marcha)=Pmot(marcha)/rot(marcha);
    end
    // escolhe a máxima potência entre todas as marchas
    [Pmax,marcha_esc] = max(Pmot(:));
    //[Tmax,marcha_esc] = max(Tmot(:));
    if  rot>MOT.w_motor_corte_max  then
        marcha_esc = n_marchas  
    elseif rot<MOT.w_motor_corte_min then
        marcha_esc = 1
    end
 //   marcha_esc = 1    
    rotacao = rot(marcha_esc);
    // potência na roda é diminuida pela eficiência da transmissão
    Proda = eval_Pmotmax(MOT,PM,marcha_esc)*TRN.eta_t;
endfunction


// INÍCIO DO PROGRAMA

pi = 3.141592;                             // pi

// // parametros do motor
MCI.P_pico = 22080/0.866;                  // P_pico  [W]
MCI.P_int = MCI.P_pico*0.15;               // P_int  [W]
MCI.P_corte_min = 3000                     // P_corte_min  [W]
MCI.w_motor_P_pico = 8000*(2*pi/60);       // w_P_pico
MCI.w_motor_T_pico = 5500*(2*pi/60);
MCI.w_motor_corte_min = 1500*(2*pi/60);
MCI.w_motor_corte_max = 9000*(2*pi/60);
MCI.w_motor_eta_otimo = MCI.w_motor_T_pico;
MCI.eta_nominal = 0.30;
MCI.eta_rot_rot_max = 0.9;
MCI.eta_alpha_rot_min = 0.9;
MCI.alpha_otimo = 0.9;

// parametros da transmissão
TRN.csi_prim = 2.8;
TRN.csi_marcha_i = [2.6 1.9 1.4 1.1 0.9];  
TRN.csi_fin = 2.6;
TRN.RRoda = 0.3;
TRN.eta_1 = 0.98;
TRN.eta_2 = 0.98;
TRN.eta_3 = 0.98;
TRN.eta_4 = 0.92;

// parametros fisicos da moto e da pista
global Par;             // estrutura global 
//Par.P_max = 22080;      // potencia maxima na roda [W]
Par.m_c = 250;          // massa do conjunto [kg]
Par.k_A = 0.35;         // fator de arrasto aerodinamico [kg/m]
Par.C_R = 0.02;         // fator de resistencia a rolagem 
Par.mu = 0.8;           // coeficiente de atrito
Par.W = -0;             // velocidade do vento [m/s]
pi = 3.141592;          // pi
Par.teta = 0/180*pi;    // angulo de aclive da pista [rad]
Par.g = 9.81;           // aceleracao da gravidade [m/s^2]
Par.p_d=1.4;            // distância entre eixos [m]
Par.p_CG=0.67;          // distância do CG ao eixo  traseiro [m]
Par.h_CG=0.72;          // altura do centro de gravidade [m]
Par.h_CP=0.95;          // altura do centro de pressao [m]

// CALCULOS ADICIONAIS
TRN.eta_t = TRN.eta_1*TRN.eta_2*TRN.eta_3*TRN.eta_4;

// CHAMADA DA FUNÇÃO DE CÁLCULO DOS COEFICIENTES DAS CURVAS DO MOTOR
PM = eval_parmot(MCI);

// CÁLCULO DOS VETORES DE POTÊNCIA MÁXIMA, POTÊNCIA RESISTIVA, MARCHA, CONSUMO E ROTAÇÃO

V=0:0.1:160/3.6;
for i=1:size(V,2)
    [Ppromax(i),m(i),w(i)]=eval_Pmaxroda(MCI,PM,TRN,V(i))
    alpha = 1;
    Ppro(i) = Ppromax(i)*alpha;
    Pcomb(i)=eval_Pcomb(MCI,PM,w(i),alpha)
    Pmot(i)=eval_Pmotmax(MCI,PM,w(i))*alpha
    Tmot(i)=Pmot(i)/w(i);
    Froda(i)= Ppro(i)/V(i)
    Pres(i) =  (Par.m_c*Par.g*sin(Par.teta) + Par.k_A*(V(i)-Par.W)^2 + Par.C_R*abs(Par.m_c*Par.g*cos(Par.teta)))*V(i);
end

// GRÁFICOS

//plot(3.6*V',Ppro/735,'k','LineWidth',3);
//plot(3.6*V',Pres/735,'k','LineWidth',3);
plot(3.6*V',Pmot/735,'k','LineWidth',3);
//plot(3.6*V',Pcomb/735,'k','LineWidth',3);
xlabel('$V\quad\text{[km/h]}$','fontsize',4);     // titulo do eixo x
ylabel('$P\quad\text{[CV]}$','fontsize',4);       // titulo do eixo y

//plot(3.6*V',Tmot,'k','LineWidth',3);
//xlabel('$V\quad\text{[km/h]}$','fontsize',4);     // titulo do eixo x
//ylabel('$T\quad\text{[Nm]}$','fontsize',4);       // titulo do eixo y

//plot(3.6*V',Froda,'k','LineWidth',3);
//xlabel('$V\quad\text{[km/h]}$','fontsize',4);     // titulo do eixo x
//ylabel('$F\quad\text{[N]}$','fontsize',4);       // titulo do eixo y

//plot(3.6*V',m,'k','LineWidth',3);
//xlabel('$V\quad\text{[km/h]}$','fontsize',4);     // titulo do eixo x
//ylabel('$\text{marcha}$','fontsize',4);           // titulo do eixo y

//plot(3.6*V',w*(60/(2*pi)),'k','LineWidth',3);
//xlabel('$V\quad\text{[km/h]}$','fontsize',4);      // titulo do eixo x
//ylabel('$\text{rotação   [rpm]}$','fontsize',4);       // titulo do eixo y

