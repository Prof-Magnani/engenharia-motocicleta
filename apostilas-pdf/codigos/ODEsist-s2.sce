// comandos iniciais ao sistema computacional
clc;            // limpa o console
clear;          // limpa as variaveis
clearglobal;    // limpa as variaveis globais


// dados fisicos da simulacao
x0 = 0;             // posição inicial [s]
V0 = 0.1;           // velocidade inicial [m/s]
t0=0;               // tempo inicial [s]
tf =100;            // tempo final [s]
dt = 0.1;           // incremento de tempo [s]

// parametros fisicos
global Par;             // estrutura global 
Par.P_max = 22080;      // potencia maxima [W]
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

// criacao de vetores
Q0 = [x0;V0];           // vetor de variáveis iniciais
t=t0:dt:tf              // vetor de instantes calculados

function f=S(t,Q)  // sistema de EDOs 
    //extracao das variaveis
    x = Q(1,1);          // x                   
    V = Q(2,1);          // V
    
    // modelo do piloto
    alpha = 1;      // acelerador
    beta_t = 0;     // freio traseiro
    beta_d = 0;     // dianteiro

    // modelo do motor
    P_max = Par.P_max;          // potencia maxima constante
    
    // modelo da pista
    teta = Par.teta;            // angulo de aclive
    mu = Par.mu;                // coeficiente de atrito
    W = Par.W;                  // velocidade do vento [m/s]

    // calculo das forcas
    F_grx = -Par.m_c*Par.g*sin(teta);   // componente x da gravidade
    F_gry = -Par.m_c*Par.g*cos(teta);   // componente y da gravidade
    F_pro = alpha*P_max/V;              // propulsao
    F_aer = -Par.k_A*(V-W)^2;           // arrasto aerodinamio
    F_rol = -Par.C_R*abs(F_gry);        // resistencia a rolagem
    
    //calculo da funcao D
    D = (F_pro+F_aer+F_rol+F_grx+beta_t*mu*F_gry + (beta_d-beta_t)*mu/Par.p_d*( -F_aer*Par.h_CP + -F_grx*Par.h_CG + F_gry*Par.p_CG)  )/(Par.m_c*(1-(beta_d-beta_t)*mu*Par.h_CG/Par.p_d));
    
    // sistema de equacoes diferenciais
    f(1)= V;                  
    f(2)= D;
endfunction 

// solucao do sistema de equacoes diferenciais
Q=ode(Q0,t0,t,S)       //funcao ODE do Scilab

// extracao dos vetores das variaveis
x=Q(1,:);
V=Q(2,:);

// graficos
plot(t,V*3.6,'k-')     // grafico [1 m/s = 3.6 km/h]
xlabel('$t\quad[s]$','fontsize',4);       // titulo do eixo x
ylabel('$V\quad[km/h]$','fontsize',4);    // titulo do eixo y




