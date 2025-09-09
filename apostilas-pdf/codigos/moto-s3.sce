// comandos iniciais ao sistema computacional
clc;            // limpa o console
clear;          // limpa as variaveis
clearglobal;    // limpa as variaveis globais
// 
//////////////////////  /////////////////////////////
// INICIO  DO PROGRAMA  <<--------- --------------- 
///////////////////////////////////////////////////
getd('.\');                                                     // carrega as funcões
[sP,sA] = get_parametros_v3();                                  // carrega os parâmetros
vet_Vseg = get_vetor_Vseg();

// SOLUÇÃO DA EDO
Q0 = [sP.x0;sP.V0];                                             // vetor de variáveis iniciais
t=sP.t0:sP.DT_num:sP.t_sim                                     // vetor de instantes calculados
Q=ode(Q0,sP.t0,t,S)                                             // funcao ODE do Scilab
tempo_L = t(find(Q(1,:)>sP.Lpista,1));                          // encontra o tempo para o deslocamento L
disp("tempo para L_pista [s]",tempo_L);                         // escreve o tempo para o deslocamento L no console

// EXTRAÇÃO DOS VETORES DE VARIÁVEIS SECUNDÁRIAS
x=Q(1,:);                               // vetor de posições encontrado pela ode()
V=Q(2,:);                               // vetor de velocidades encontrado pela ode()
nt = size(t,2)
for i=1:size(t,2)
    [lixo,sE]=S(t(i),Q(:,i));            // pega a estrutura E
    // extraídos da estrutura E
    alpha(1,i) = sE.alpha;               // abertura do acelerador
    beta_d(1,i) = sE.beta_d;             // uso do freio dianteiro
    beta_t(1,i) = sE.beta_t;             // uso do freio traseiro
    mu(1,i) = sE.mu                      // atrito
    W(1,i) = sE.W                        // velocidade do vento
    RCurvatura(1,i) = sE.RCurvatura      // raio de curvatura
    a(1,i) = sE.a;                       // aceleracao
    a_min(1,i) = sE.a_min;               // aceleracao minima
    a_max(1,i) = sE.a_max;               // aceleracao maxima
    Vdes(1,i) = get_Vdes(x(1,i));       // velocidade desejada
    Vmax(1,i) = get_Vseg(vet_Vseg,x(1,i));   // velocidade de seguranca
    mar(1,i) = sE.mar;                   // marcha
    rot(1,i) = sE.rot;                   // rotação do motor
    eta(1,i) = sE.eta;                   // eficiência do cilindro
    teta(1,i) = sE.teta;                 // ângulo de aclive
    Fgrx(1,i) = sE.Fgrx;                 // componente x da força da gravidade
    Fgry(1,i) = sE.Fgry;                 // componente y da força da gravidade
    Fpro(1,i) = sE.Fpro;                 // força de propulsão
    Faer(1,i) = sE.Faer;                 // força de arrasto aerodinâmico
    Frol(1,i) = sE.Frol;                 // força de resistência à rolagem
    Nrd(1,i) = sE.N_d;                   // normal dianteira
    Nrt(1,i) = sE.N_t;                   // normal traseira
    u_ad_d(1,i) = sE.u_ad_d;             // utilização da aderência dianteira
    u_ad_t(1,i) = sE.u_ad_t;             // utilização da aderência traseira
    u_no_d(1,i) = sE.u_no_d;             // utilização da normal dianteira
    u_no_t(1,i) = sE.u_no_t;             // utilização da normal traseira
    Ffrn_t(1,i) = sE.Ffrn_t;             // força de frenagem traseira
    Ffrn_d(1,i) = sE.Ffrn_d;             // força de frenagem dianteira
    Pcomb(1,i) = sE.Pcomb;               // potência do combustível
    Pint(1,i) = sE.Pint;                 // potência interna dissipada
    Pmaxroda(1,i) = sE.Pmaxroda;         // potência máxima na roda
    // grandezas calculadas
    Fine(1,i) = -sA.m_c*a(i);           // força de inércia
    Pine(1,i) = Fine(1,i)*V(i);         // potência de inércia
    Pgrx(1,i) = Fgrx(1,i)*V(i);         // potência da resitência grav x
    Ppro(1,i) = Fpro(1,i)*V(i);         // potência da resitência grav y
    Paer(1,i) = Faer(1,i)*V(i);         // potência aerodinâmica
    Prol(1,i) = Frol(1,i)*V(i);         // potência de rolagem
    Pfrnt(1,i) = Ffrn_t(1,i)*V(i);      // potência do freio traseiro
    Pfrnd(1,i) = Ffrn_d(1,i)*V(i);      // potência do freio dianteiro
    eta2(1,i) = Ppro(1,i)/sA.eta_t/sE.Pcomb; // eficiência do motor inteiro
end

// CÁLCULO DAS ENERGIAS  E = int(Pdt)
SAI.Ecomb = 0;
SAI.Eint = 0;
SAI.Emotor = 0;
SAI.Epro = 0;
SAI.alt = 0;
SAI.Eine = 0;
SAI.Egrx = 0;
SAI.Epro = 0; 
SAI.Eaer = 0;
SAI.Erol = 0;
SAI.Efrnt = 0;
SAI.Efrnd = 0;
for i=2:size(t,2)
    dt = (t(i)-t(i-1));
    SAI.Ecomb= SAI.Ecomb + (Pcomb(1,i)+Pcomb(1,i-1))/2*dt;              // energia do combustível
    SAI.Eint = SAI.Eint + Pint(1,i)*dt;                                 // energia dissipada internamente
    SAI.Emotor = SAI.Emotor + (Pcomb(1,i)*eta(1,i)+Pcomb(1,i-1)*eta(1,i-1))/2*dt - Pint(1,i)*dt;    // energia mecânica do motor
    SAI.alt = SAI.alt + (x(i)-x(i-1))*sin((teta(i)+teta(i-1))/2);       // altura
    SAI.Eine = SAI.Eine - (Pine(1,i)+Pine(1,i-1))/2*dt;                 // energia cinética
    SAI.Egrx = SAI.Egrx - (Pgrx(1,i)+Pgrx(1,i-1))/2*dt;                 // energia gravitacional
    SAI.Epro = SAI.Epro + (Ppro(1,i)+Ppro(1,i-1))/2*dt;                 // energia de propulsão
    SAI.Eaer = SAI.Eaer - (Paer(1,i)+Paer(1,i-1))/2*dt;                 // energia dissipada pelo arrasto aerodinâmico
    SAI.Erol = SAI.Erol - (Prol(1,i)+Prol(1,i-1))/2*dt;                 // energia dissipada pela resistência a rolagem
    SAI.Efrnt = SAI.Efrnt - (Pfrnt(1,i)+Pfrnt(1,i-1))/2*dt;             // energia dissipada pelo freio traseiro
    SAI.Efrnd = SAI.Efrnd - (Pfrnd(1,i)+Pfrnd(1,i-1))/2*dt;             // energia dissipada pelo freio dianteiro
end

// CONTABILIDADE DO TRAJETO COMPLETO
SAI.Vmax_kph = max(V)*3.6;                                  // velocidade máxima [km/h]
SAI.L0p99km = x(find(V*3.6>SAI.Vmax_kph*0.99,1))/1000;      // distância para alcançar 99% da velocidade máxima 
SAI.tV30seg = t(find(V*3.6>30,1));                          // tempo para alcançar 30 km/h
SAI.tV60seg = t(find(V*3.6>60,1));                          // tempo para alcançar 60 km/h
SAI.tV100seg = t(find(V*3.6>100,1));                        // tempo para alcançar 100 km/h
SAI.Vrefkph = sP.Vref*3.6;                                  // Vref
SAI.tVrefseg = t(find(V>sP.Vref,1));                        // tempo para alcançar Vref
SAI.LVrefkm = x(find(V>sP.Vref,1))/1000;                    // distância para alcançar Vref
SAI.L_km = (x(nt)-x(1))/1000;                               // deslocamento total  [km]
SAI.tempo = t(nt)-t(1);                                     // tempo total  [s]
SAI.C_litros =  SAI.Ecomb/sP.PC_comb/sP.rho_comb*1000;      // consumo de combustível [litros]
SAI.eco_kmplitro = SAI.L_km/SAI.C_litros;                   // economia [km/litro]

// CÁLCULO DOS PERCENTUAIS
PERC.gases_quentes = (SAI.Ecomb-SAI.Emotor-SAI.Eint)/SAI.Ecomb;     // percentual dos gases quentes
PERC.mec_int = (SAI.Eint)/SAI.Ecomb;                                // percentual da dissipação interna
PERC.trans = (SAI.Emotor - SAI.Epro)/SAI.Ecomb;                     // percentual da dissipação na transmissão
PERC.freios = (SAI.Efrnt + SAI.Efrnd)/SAI.Ecomb;                    // percentual de dissipação nos freios
PERC.aer = SAI.Eaer/SAI.Ecomb;                                      // percentual dissipado pelo arrasto aerodinâmico
PERC.rol = SAI.Erol/SAI.Ecomb;                                      // percentual dissipado pela resistência a rolagem
PERC.grx = SAI.Egrx/SAI.Ecomb;                                      // percentual usado pela energia potencial gravitacional
PERC.ine = SAI.Eine/SAI.Ecomb;                                      // percenutal usado pela energia cinética
PERC.soma = PERC.gases_quentes +  PERC.mec_int +  PERC.trans +  PERC.freios +   PERC.aer + PERC.rol + PERC.grx + PERC.ine;  // soma   dos percentuais
PERC.efic_prop = SAI.Epro/SAI.Ecomb;                                // eficiência de propusao
PERC.efic_transp =  (PERC.grx + PERC.ine)*sP.m_p/sA.m_c;            // eficiência de transporte

// MONITORAMENTO DE PROBLEMAS
PROB.max_u_ad_d=max(u_ad_d);               // utilização máxima da aderência dianteira
PROB.max_u_ad_t=max(u_ad_t);               // utilização máxima da aderência traseira
PROB.max_u_no_d=max(u_no_d);               // utilização máxima da normal dianteira
PROB.max_u_no_t=max(u_no_t);               // utilização máxima da normal traseira

// SAÍDA DOS RESULTADOS PRINCIPAIS
disp(SAI)
disp(PERC)
disp(PROB)

// GRÁFICO  ÚNICO
//plot2d(x,V*3.6,style=color("black"))
//xlabel('$x\quad[m]$','fontsize',4);      
//ylabel('$V\quad[km/h]$','fontsize',4);
//h = gca(); // get current axes
//h.children(1).children(1).thickness = 3;
//h.children(1).children(1).line_style = 1;   //1: contínua, 7: pontilhada, 8: tracejada
//h.data_bounds = [0, 0 ; 500, 140];
//h.tight_limits=["on","on"];        
//xstring(450,123,'$1$')
//gce().font_size = 3;
//xstring(450,107,'$2$')
//gce().font_size = 3;
//xstring(450,76,'$3$')
//gce().font_size = 3;

// QUATRO SUB-GRÁFICOS
subplot(2,2,1)
xlabel('$x\quad[m]$','fontsize',4);      
ylabel('$V\quad[km/h]$','fontsize',4);
plot2d(x,Vdes*3.6,style=color("gray"))  
h = gca(); // get current axes
h.children(1).children(1).thickness = 3;
h.children(1).children(1).line_style = 1;   //1: contínua, 7: pontilhada, 8: tracejada
plot2d(x,V*3.6,style=color("black"))
h = gca(); // get current axes
h.children(1).children  (1).thickness = 3;
h.children(1).children(1).line_style = 1;   //1: contínua, 7: pontilhada, 8: tracejada


subplot(2,2,2)
xlabel('$x\quad[m]$','fontsize',4);       
ylabel('$u_{ad}$','fontsize',4);    
plot2d(x,u_ad_t,style=color("gray"))                        
h = gca(); // get current axes
h.children(1).children(1).thickness = 3;
h.children(1).children(1).line_style = 1;   //1: contínua, 7: pontilhada, 8: tracejada
plot2d(x,u_ad_d,style=color("black"))                        
h = gca(); // get current axes
h.children(1).children(1).thickness = 3;
h.children(1).children(1).line_style = 1;   //1: contínua, 7: pontilhada, 8: tracejada

subplot(2,2,3)
xlabel('$x\quad[m]$','fontsize',4);       
ylabel('$u_{no}$','fontsize',4);    
plot2d(x,u_no_t,style=color("gray"))                        
h = gca(); // get current axes
h.children(1).children(1).thickness = 3;
h.children(1).children(1).line_style = 1;   //1: contínua, 7: pontilhada, 8: tracejada
plot2d(x,u_no_d,style=color("black"))                        
h = gca(); // get current axes
h.children(1).children(1).thickness = 3;
h.children(1).children(1).line_style = 1;   //1: contínua, 7: pontilhada, 8: tracejada

subplot(2,2,4)
xlabel('$x\quad[m]$','fontsize',4); 
ylabel('$\alpha,\beta$','fontsize',4);    
plot2d(x,beta_t,style=color("gray"))                        
h = gca(); // get current axes
h.children(1).children(1).thickness = 3;
h.children(1).children(1).line_style = 1;   //1: contínua, 7: pontilhada, 8: tracejada
plot2d(x,beta_d,style=color("black"))                        
h = gca(); // get current axes
h.children(1).children(1).thickness = 3;
h.children(1).children(1).line_style = 1;   //1: contínua, 7: pontilhada, 8: tracejada
plot2d(x,alpha,style=color("black"))                        
h = gca(); // get current axes
h.children(1).children(1).thickness = 3;
h.children(1).children(1).line_style = 7;   //1: contínua, 7: pontilhada, 8: tracejada
