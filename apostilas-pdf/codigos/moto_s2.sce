// comandos iniciais ao sistema computacional
clc;            // limpa o console
clear;          // limpa as variaveis
clearglobal;    // limpa as variaveis globais


function [D,E]=A(t,Q)
    //extracao das variaveis
    x = Q(1,1);          // x                   
    V = Q(2,1);          // V

    // modelo do piloto
    E.alpha = 1;      // acelerador
    if x>359 then
        E.alpha = 1 ;
    end
    E.beta_t = 0;     // freio traseiro
    E.beta_d = 0;     // dianteiro
    
     // modelo do motor
    [E.Pmaxroda,E.mar,E.rot]=eval_Pmaxroda(gMCI,gPM,gTRN,V) // calcula a Pmaxroda
    //E.Pmaxroda = gMCI.P_max_roda_constante;          // potencia maxima constante 
    
    // modelo da pista
    teta = gPar.teta;            // angulo de aclive
    mu = gPar.mu;                // coeficiente de atrito
    W = gPar.W;                  // velocidade do vento [m/s]

    // calculo das forcas
    E.Fgrx = -gPar.m_c*gPar.g*sin(teta);   // componente x da gravidade
    E.Fgry = -gPar.m_c*gPar.g*cos(teta);   // componente y da gravidade
    E.Fpro = E.alpha*E.Pmaxroda/V;           // propulsao
    E.Faer = -gPar.k_A*(V-W)^2;            // arrasto aerodinamio
    E.Frol = -gPar.C_R*abs(E.Fgry);        // resistencia a rolagem

    //calculo da funcao D
    D = (E.Fpro+E.Faer+E.Frol+E.Fgrx+E.beta_t*mu*E.Fgry + (E.beta_d-E.beta_t)*mu/gPar.p_d*( -E.Faer*gPar.h_CP + -E.Fgrx*gPar.h_CG + E.Fgry*gPar.p_CG)  )/(gPar.m_c*(1-(E.beta_d-E.beta_t)*mu*gPar.h_CG/gPar.p_d));

    // Outras variáveis extraídas
    E.teta = teta;                                  // aclive
    E.a = D;                                        // aceleração
    E.Pcomb=eval_Pcomb(gMCI,gPM,E.rot,E.alpha)      // Pcomb
    E.Fine = -E.a*gPar.m_c;                         // inercia
    E.Nrd = (E.Fine*gPar.h_CG + E.Faer*gPar.h_CP + E.Fgrx*gPar.h_CG - E.Fgry*gPar.p_CG)/gPar.p_d;
                                                    // normal dianteira
    E.Nrt = -E.Nrd - E.Fgry;                        // normal traseira
    E.eta = eval_efic (gMCI,gPM,E.rot,E.alpha);     // eficiência do motor
    E.Ffrnt = -E.beta_t*mu*E.Nrt;                   // frenagem traseira
    E.Ffrnd = -E.beta_d*mu*E.Nrd;                   // frenagem dianteira
endfunction

function [f,vet]=S(t,Q)  // sistema de EDOs 
    f(1)= Q(2,1);              
    f(2)= A(t,Q);
endfunction 

// INICIO  DO PROGRAMA  <<------------------------

// dados da simulacao
pi = 3.141592;       // pi
x0 = 0;             // posição inicial [s]
V0 = 0.01/3.6;           // velocidade inicial [m/s]
t0=0;               // tempo inicial [s]
L =2000;           // comprimento desejado 
dt = 0.1;           // incremento de tempo [s]
Vref = 127.85/3.6;         // Velocidade de referência [km/h]
TF_num=1000000;        // tempo final na primeira simulação para encontrar o tempo para L
DT_num = 0.1;          // dt na primeira simulação para encontrar o tempo para L

// carrega as funcões
getd('.\');

//estruturas globais que serão usadas na função S
global gPar;
global gMCI;
global gTRN;
global gPM;             

// carrega os parâmetros
[gPar,gMCI,gPM, gTRN] = get_parametros();

// criacao de vetores para encontrar o t relativo a L
Q0 = [x0;V0];           // vetor de variáveis iniciais
t=t0:DT_num:TF_num             // vetor de instantes calculados
Q=ode(Q0,t0,t,S)       //funcao ODE do Scilab
tempo_L = t(find(Q(1,:)>L,1));  // encontra o tempo para o deslocamento L
clear t; clear Q;       // limpa os vetores

// criacao de vetores
t=t0:dt:tempo_L         // vetor de instantes calculados para a nova distância

// solução da ODE
Q=ode(Q0,t0,t,S)       //funcao ODE do Scilab

// extracao dos vetores das variaveis
x=Q(1,:);                       // vetor de posições
V=Q(2,:);                       // vetor de velocidades
nt = size(t,2)
for i=1:size(t,2)
    [lixo,E]=A(t(i),Q(:,i));     // pega a estrutura E
    alpha(1,i) = E.alpha;               // abertura do acelerador
    beta_d(1,i) = E.beta_d;             // uso do freio dianteiro
    beta_g(1,i) = E.beta_t;             // uso do freio traseiro
    a(1,i) = E.a;                       // aceleracao
    mar(1,i) = E.mar;                   // marcha
    rot(1,i) = E.rot;                   // rotação do motor
    eta(1,i) = E.eta;                   // eficiência do cilindro
    teta(1,i) = E.teta;                 // ângulo de aclive
    Fgrx(1,i) = E.Fgrx;                 // componente x da força da gravidade
    Fgry(1,i) = E.Fgry;                 // componente y da força da gravidade
    Fpro(1,i) = E.Fpro;                 // força de propulsão
    Faer(1,i) = E.Faer;                 // força de arrasto aerodinâmico
    Frol(1,i) = E.Frol;                 // força de resistência à rolagem
    Nrd(1,i) = E.Nrd;                   // normal dianteira
    Nrt(1,i) = E.Nrt;                   // normal traseira
    Ffrnt(1,i) = E.Ffrnt;               // força de frenagem traseira
    Ffrnd(1,i) = E.Ffrnd;               // força de frenagem dianteira
    Pcomb(1,i) = E.Pcomb;               // potência do combustível
    Pmaxroda(1,i) = E.Pmaxroda;         // potência máxima na roda
    Fine(1,i) = -gPar.m_c*a(i);         // força de inércia
    Pine(1,i) = Fine(1,i)*V(i);         // potência de inércia
    Pgrx(1,i) = Fgrx(1,i)*V(i);         // potência da resitência grav x
    Ppro(1,i) = Fpro(1,i)*V(i);         // potência da resitência grav y
    Paer(1,i) = Faer(1,i)*V(i);         // potência aerodinâmica
    Prol(1,i) = Frol(1,i)*V(i);         // potência de rolagem
    Pfrnt(1,i) = Ffrnt(1,i)*V(i);       // potência do freio traseiro
    Pfrnd(1,i) = Ffrnd(1,i)*V(i);       // potência do freio dianteiro
    eta2(1,i) = Pmaxroda(1,i)/gTRN.eta_t/E.Pcomb; // eficiência do motor inteiro
end

// contabilidade

SAI.Vmax_kph = max(V)*3.6;
SAI.L0p99km = x(find(V*3.6>SAI.Vmax_kph*0.99,1))/1000;
SAI.tV30seg = t(find(V*3.6>30,1));
SAI.tV60seg = t(find(V*3.6>60,1));
SAI.tV100seg = t(find(V*3.6>100,1));
SAI.tVrefseg = t(find(V>Vref,1));
SAI.LVrefkm = x(find(V>Vref,1))/1000;
SAI.Vrefkph = Vref*3.6;
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
// CÁLCULO DAS ENERGIAS  E = int(Pdt)
for i=2:size(t,2)
    dt = (t(i)-t(i-1));
    SAI.Ecomb= SAI.Ecomb + (Pcomb(1,i)+Pcomb(1,i-1))/2*dt;
    SAI.Eint = SAI.Eint + gMCI.P_int*dt; 
    SAI.Emotor = SAI.Emotor + (Pcomb(1,i)*eta(1,i)+Pcomb(1,i-1)*eta(1,i-1))/2*dt - gMCI.P_int*dt;
    SAI.alt = SAI.alt + (x(i)-x(i-1))*sin((teta(i)+teta(i-1))/2);
    SAI.Eine = SAI.Eine - (Pine(1,i)+Pine(1,i-1))/2*dt;
    SAI.Egrx = SAI.Egrx - (Pgrx(1,i)+Pgrx(1,i-1))/2*dt;
    SAI.Epro = SAI.Epro + (Ppro(1,i)+Ppro(1,i-1))/2*dt; 
    SAI.Eaer = SAI.Eaer - (Paer(1,i)+Paer(1,i-1))/2*dt;
    SAI.Erol = SAI.Erol - (Prol(1,i)+Prol(1,i-1))/2*dt;
    SAI.Efrnt = SAI.Efrnt - (Pfrnt(1,i)+Pfrnt(1,i-1))/2*dt;
    SAI.Efrnd = SAI.Efrnd - (Pfrnd(1,i)+Pfrnd(1,i-1))/2*dt;
end
SAI.L_km = (x(nt)-x(1))/1000;                   // deslocamento total  [km]
SAI.tempo = t(nt)-t(1);                         // tempo total  [s]
SAI.C_litros =  SAI.Ecomb/gMCI.PCvol*1000;      // consumo de combustível [litros]
SAI.eco_kmplitro = SAI.L_km/SAI.C_litros;       // economia [km/litro]

// percentuais
PERC.gases_quentes = (SAI.Ecomb-SAI.Emotor-SAI.Eint)/SAI.Ecomb;
PERC.mec_int = (SAI.Eint)/SAI.Ecomb;
PERC.trans = (SAI.Emotor - SAI.Epro)/SAI.Ecomb;
PERC.freios = (SAI.Efrnt + SAI.Efrnd)/SAI.Ecomb;
PERC.aer = SAI.Eaer/SAI.Ecomb;
PERC.rol = SAI.Erol/SAI.Ecomb;
PERC.grx = SAI.Egrx/SAI.Ecomb;
PERC.ine = SAI.Eine/SAI.Ecomb;
PERC.soma = PERC.gases_quentes +  PERC.mec_int +  PERC.trans +  PERC.freios +   PERC.aer + PERC.rol + PERC.grx + PERC.ine;
EFIC.prop = SAI.Epro/SAI.Ecomb;     // eficiência de propusao
EFIC.transp =  (PERC.grx + PERC.ine)*gPar.m_p/gPar.m_c;     // eficiência de transporte

// saída
disp(SAI)
disp(PERC)
disp(EFIC)

// graficos
//plot(t,V*3.6,'k','LineWidth',3)                        // grafico
//xlabel('$t\quad[s]$','fontsize',4);       // titulo do eixo x
//ylabel('$V\quad[km/h]$','fontsize',4);    // titulo do eixo y

//plot(x/1000,V*3.6,'k','LineWidth',3)                        // grafico
//xlabel('$x\quad[km]$','fontsize',4);       // titulo do eixo x
//ylabel('$V\quad[km/h]$','fontsize',4);    // titulo do eixo y


//plot(t,Ppro/735,'k','LineWidth',3)                        // grafico
//plot(t,-Pine/735,'k','LineWidth',3)                        // grafico
//plot(t,-Paer/735,'k','LineWidth',3)                        // grafico
//plot(t,-Prol/735,'k','LineWidth',3)                        // grafico
//plot(t,-Pgrx/735,'k','LineWidth',3)                        // grafico
//xlabel('$t\quad[s]$','fontsize',4);       // titulo do eixo x
//ylabel('$P\quad[CV]$','fontsize',4);    // titulo do eixo y

//plot(t,Fpro,'k','LineWidth',3)                    // grafico
plot(t,Nrt*gPar.mu,'k','LineWidth',3)                    // grafico
//plot(t,Fine,'k','LineWidth',3)                         // grafico
//plot(t,Faer,'k','LineWidth',3)                         // grafico
//plot(t,Frol,'k','LineWidth',3)                         // grafico
plot(t,Nrd,'b','LineWidth',3)                         // grafico
//plot(t,Nrt,'k','LineWidth',3)                        // grafico
xlabel('$t\quad[s]$','fontsize',4);       // titulo do eixo x
ylabel('$F\quad[N]$','fontsize',4);    // titulo do eixo y

// MUDANÇAS ESTRUTURAIS
//   Função S foi para cima
//   As funções estao em arquivos separados
//   Os parâmetros são carregados na função get_parametros
//   Mudança de Par.P_max  para gPar.P_max_roda_constante
//   Função S chama a função eval_DFP
//   Variáveis F_... e P_... na estrutura E
//   Outras variáveis extraídas

//  MUDANÇA FÍSICA
//  Chamada da função eval_Pmaxroda(gMCI,gPM,gTRN,V)

// OUTRAS MUDANÇAS
//    FIS.PCvol
//    Cálculo do consumo
