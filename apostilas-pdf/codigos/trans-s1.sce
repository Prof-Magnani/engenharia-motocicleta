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
    Pmax = PM.a(1) + PM.a(2)*rot + PM.a(3)*rot^2 + PM.a(4)*rot^3;
endfunction

// FUNÇÃO EFICIÊNCIA DO MOTOR
function eficiencia=eval_efic (MOT,PM,rot,a)
    w_norm = (rot-MOT.w_motor_corte_min)/(MOT.w_motor_corte_max - MOT.w_motor_corte_min);
    eficiencia = MOT.eta_nominal*(PM.b(1) + PM.b(2)*w_norm + PM.b(3)*w_norm^2)*(PM.c(1) + PM.c(2)*a + PM.c(3)*a^2);
endfunction

// FUNÇÃO POTÊNCIA DO COMBUSTÍVEL
function Pcomb=eval_Pcomb (MOT,PM,rot,a)
    Pcomb = (a*eval_Pmotmax(MOT,PM,rot) + MOT.P_int)./eval_efic(MOT,PM,rot,a)
endfunction


// INÍCIO DO PROGRAMA

// ESTRUTURA COM OS PARÂMETROS DO MOTOR
pi = 3.141592;                                 // pi
MCI.P_pico = 22080/0.866;                      // P_pico  [W]
MCI.P_int = MCI.P_pico*0.15;                   // P_int  [W]
MCI.P_corte_min = 3000                         // P_corte_min  [W]
MCI.w_motor_P_pico = 8000*(2*pi/60);           // w_P_pico
MCI.w_motor_T_pico = 5500*(2*pi/60);           // w_T_pico
MCI.w_motor_corte_min = 1500*(2*pi/60);        // w_corte_min
MCI.w_motor_corte_max = 9000*(2*pi/60);        // w_corte_max
MCI.w_motor_eta_otimo = 5500*(2*pi/60);        // rotação de efic. máxima     

MCI.eta_nominal = 0.40;         // eficiência nominal do motor
MCI.eta_rot_rot_max = 0.9;      // efic. eta_w na rot. máx
MCI.eta_alpha_rot_min = 0.9;    // efic. eta_alpha na rot. min
MCI.alpha_otimo = 0.9;          // alpha ótimo

// CHAMADA DA FUNÇÃO DE CÁLCULO DOS COEFICIENTES DAS CURVAS DO MOTOR
PM = eval_parmot(MCI)

// CALCULO EM FUNÇÃO DE w
w = MCI.w_motor_corte_min:10:MCI.w_motor_corte_max
alpha = 1;
eta = eval_efic(MCI,PM,w,alpha)
P = eval_Pmotmax(MCI,PM,w)
T = P./w;
eta = eval_efic(MCI,PM,w,alpha)
Pcomb=eval_Pcomb(MCI,PM,w,alpha)

// GRÁFICOS EM FUNÇÃO DE w
//plot(w,P,'k','LineWidth',3);
//label('$\omega\quad\text{[rad/s]}$','fontsize',4);    // titulo do eixo x
//ylabel('$P_{\text{max}}\quad\text{[W]}$','fontsize',4);       // titulo do eixo y

//plot(w*60/(2*pi),P,'k','LineWidth',3);
//xlabel('$\omega\quad\text{[rpm]}$','fontsize',4);    // titulo do eixo x
//ylabel('$P_{\text{max}}\quad\text{[W]}$','fontsize',4);       // titulo do eixo y

//plot(w,eta,'k','LineWidth',3)
//xlabel('$\omega\quad\text{[rad/s]}$','fontsize',4);    // titulo do eixo x
//ylabel('$\eta_{m}$','fontsize',4);       // titulo do eixo y

//plot(w*60/(2*pi),eta/MCI.eta_nominal,'k','LineWidth',3)
//xlabel('$\omega\quad\text{[rad/s]}$','fontsize',4);    // titulo do eixo x
//ylabel('$\eta_{w}$','fontsize',4);       // titulo do eixo y

plot(w,Pcomb,'k','LineWidth',3);
xlabel('$\omega\quad\text{[rad/s]}$','fontsize',4);    // titulo do eixo x
ylabel('$P_{comb}\quad\text{[W]}$','fontsize',4);      //  titulo do eixo y

// CALCULO EM FUNÇÃO DE alpha
//alpha = 0:0.05:1;
//w = 9000*(2*pi/60);
//eta = eval_efic(MCI,PM,w,alpha)
//Pcomb=eval_Pcomb(MCI,PM,w,alpha)

// GRÁFICOS EM FUNÇÃO DE alpha
//plot(alpha,eta,'k','LineWidth',3);
//xlabel('$\alpha$','fontsize',4);    // titulo do eixo x
//ylabel('$\eta_{m}$','fontsize',4);       // titulo do eixo y

//plot(alpha,eta/MCI.eta_nominal,'k','LineWidth',3);
//xlabel('$\alpha$','fontsize',4);    // titulo do eixo x
//ylabel('$\eta_{\alpha}$','fontsize',4);       // titulo do eixo y

//plot(alpha,Pcomb,'k','LineWidth',3);
//xlabel('$\alpha$','fontsize',4);                    // titulo do eixo x
//ylabel('$P_{comb}\quad\text{[W]}$','fontsize',4);       // titulo do eixo y
