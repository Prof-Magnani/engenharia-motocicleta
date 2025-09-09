clc;            // limpa o console
clear;          // limpa as variaveis
clearglobal;    // limpa as variaveis globais

// Dados
x0 = 0;             // posição inicial [m]]
V0 = 0.1;           // velocidade inicial [m/s]]
t0=0;               // tempo inicial [s]
tf =10;             // tempo final [s]
dt = 0.1;           // incremento de tempo[s]

global Par;             // estrutura global
Par.P_max = 22080;      // potencia maxima [W]
Par.F_res_med = -222;   // forca resistiva media [N]
Par.m_c = 250;          // massa do conjunto [kg]

// Inicalizacao
Q0 = [x0;V0];           // vetor de variáveis iniciais
t=t0:dt:tf              // vetor de instantes calculados

// Sistema de EDOs
function f=S(t,Q)  // sistema de EDOs
    x = Q(1,1);          // extração da variável x                   
    V = Q(2,1);          // extração da variável V
    
   //calculo da funcao D
    D =  (Par.P_max/V + Par.F_res_med)/Par.m_c;                                  
    
    // sistema de equacoes diferenciais
    f(1)= V;                  
    f(2)= D;  
endfunction 

// Solver
Q=ode(Q0,t0,t,S)       //funcao ODE do Scilab

// extracao dos vetores das variaveis
x=Q(1,:);
V=Q(2,:);

// Grafico
plot(t,V*3.6,'k-')     // grafico [1 m/s = 3.6 km/h]
xlabel('$t\quad[s]$','fontsize',4);       // titulo do eixo x
ylabel('$V\quad[km/h]$','fontsize',4);    // titulo do eixo y
