% Suorat ohjeet merkitty tähdillä.
% Käytä kuitenkin omia aivojasi, jos uskot ymmärtäväsi asian, on varmasti
% nopeampaa kirjoittaa suoraan oma ratkaisu kuin yrittää muokata minun
% koodiani.
%
% *************************************************************************
% * Täytä näissä kohdissa mainitut muuttujat
% *************************************************************************
%
% Yleinen löpinä normaaleilla kommenteilla

%--------------------------------------------------------------------------
%  Tehtävä 1)
% 
%  Hae vähintään kahden kameran kameramatriisit käyttäen tietoa
%  projisoituneiden 3D pisteiden alkuperäisistä koordinaateista.
%
%  Tarkempi selite:
%
%  Kameramatriisit P ovat 3x4 matriiseita muotoa P = K*[R -R*C] jossa
%  K on kalibrointimatriis, R rotaatiomatriisi ja C kameran
%  paikka/polttopiste.
%
%  Jos nämä olisivat tunnettuja (kalibrointimatriisi kameran kalibroimalla,
%  R ja C vaikkapa viivottimella) voitaisiin P laskea suoraan. Nyt
%  kuitenkin tunnemme vain 3D pisteiden paikat X=[X Y Z W]' (esim mitattu 
%  laser-skannerilla tai laskettu kappaleen piirustuksista). Näistä 3D 
%  pisteistä  on otettu kuva, jolloin 3D pisteet ovat projisoituneet 
%  kameran kennolle 2D pisteiksi x=[x y w]'.
%
%  Voimme mitata näiden projisoituneiden pisteiden paikat kennolla (esim
%  käyttäen MATLABin funktiota 'ginput') ja tämän jälkeen laskea 3x4
%  matriisin P joka täyttää yhtälön 
%  x = P*X.
%
%  Tarkastellaan nyt vain yhtä pisteparia X ja x.
%
%  Luennolta muistamme että ratkaisu on kertoa vasemmalta puolelta
%  x:n ristitulomatriisilla [x]_x jolloin yhtälöstä saadaan:
%  [x]_x*x = [x]_x*P*X = 0 (koska vektorin ristitulo itsensä kanssa on 0-
%  vektori).
%
%  Siis:
%  [x]_x*P*X = <3x1 matriisi> = 
%  [P_{3,4} W y - P_{2,1} X w - P_{2,2} Y w - P_{2,4} W w + P_{3,1} X y - P_{2,3} Z w + P_{3,2} Y y + P_{3,3} Z y
%   P_{1,4} W w + P_{1,1} X w - P_{3,4} W x + P_{1,2} Y w - P_{3,1} X x + P_{1,3} Z w - P_{3,2} Y x - P_{3,3} Z x
%   P_{2,4} W x + P_{2,1} X x - P_{1,4} W y - P_{1,1} X y + P_{2,2} Y x - P_{1,2} Y y + P_{2,3} Z x - P_{1,3} Z y]
%
%  Havaitaan että [x]_x*P*X voidaan järjestää muotoon
%  A*p
%  =
%  A*[p_{1,1}; p_{1,2}; p_{1,3}; p_{1,4}; p_{2,1}; p_{2,2}; p_{2,3}; p_{2,4} p_{3,1}; p_{3,2}; p_{3,3}; p_{3,4}]
%  = [0;0;0]
%  (eli A*<etsitty matriisi P purettuna pystyvektoriksi> = <nollavektori>
%
%  Hieman yllä olevaa matriisia tuijotettua havaitaan että A on 3x12 ja
%  muotoa:
%
%  A = [0 0 0 0 -X*w -Y*w -Z*w -W*w X*y Y*y Z*y W*y;
%       X*w Y*w Z*w W*w 0 0 0 0 -X*x -Y*x -Z*x -W*x;
%       -X*y -Y*y -Z*y -W*y X*x Y*x Z*x W*x 0 0 0 0]
%
%  (HUOM: yllä olevassa matriisissa X, Y, Z ja W ovat homogeenisen 3D
%  pisteen/vektorin X komponentit ja x, y ja w 2D pisteen/vektorin x
%  komponentit).
%
%  Eli jokaiselle X <-> x pisteparille saadaan laskettua matriisi A.
%  Kasaamalla nämä matriisit päällekkäin 3*n x 12 matriisiksi (jossa n
%  havaittujen pisteiden lukumäärä) voimme etsiä vektorin p.
%
%  Taasen luennoilta muistamme että p saadaan laskemalla matriisin ydin (
%  tai vektori joka on lähinnä tätä). Tämä onnistuu SVD:llä.
% 
%--------------------------------------------------------------------------


% 3D pisteiden koordinaatit homogeenisissä koordinaateissa
% (yksikkönä [mm])
% Katso visualisointi tiedostosta '3d_pisteet.png'
% 1) Shakkilaudan vasen yläkulma
% 2) Shakkilaudan vasen alakulma
% 3) V:n kärki
% 4) Leikkitraktorin lampun oikea yläkulma
% 5) Helikopterin laskeutumisjalaksen pallon kärki
% 6) Rubikin kuution vasen yläkulma (keltaisen tarran vasen alakulma)
% 7) Punaisen vaahtomuovipalan vasen yläkulma
% 8) Vihreän vaahtomuovipalan sisäkulma
% 9) Puhin kulmakarvan kärki
X = [0     0   -87   -53  -140  -181   -49  -134  -230
     0   112    18   160   171     5   -82  -141    96
     0     0    13    51     3    30    64    52    94
     1     1     1     1     1     1     1     1     1];
 
% Kameralle projisoituneet pisteet X (nyt siis 2D) homogeenisissä koordinaateissa
% (yksikkön [pixel])
%
% *************************************************************************
% * Käytä esim 'ginput' komentoa projisotuneiden pisteiden hakemisteen 
% * (tai datatip) muuttujaan x1
% *************************************************************************
% 
% x1 = [...
%       ...
%       1 1 1 1 1 1 1 1 1];

% Nyt jokaiselle pisteelle voidaan muodostaa matriisi A jotka päällekäin
% kasaamalla saadaan matriisi A_full
%
% *************************************************************************
% * Muodosta loopissa matriisi A tehtävänannon esimerkin avulla. Loopin
% * jälkeen sinulla tulee olla matriisi A_full joka koostuu päällekäin
% * kasatuista matriiseista A.
% *************************************************************************
% 
% nump = size(x1,2); %Pisteiden lukumäärä
% A_full = []; %Tähän päällekkäin kasatut matriisit A
% for ii = 1:nump
%    A = [...]
%    A_full = [A_full;A];
% end

% Nyt voimme etsiä vektorin p joka minimoi yhtälön A_full*p. Käytetään
% SVD:tä
% 
% *************************************************************************
% * Hae p käyttäen SVD:tä
% *************************************************************************
%
% [U S V] = svd(A_full, 0);
% p1 = ...; %Oikeanpuoleinen singulaarivektori joka vastaa pienintä
%           %singulaariarvoa (singulaariarvojen järjestys mainittu 
%           %'help svd'
% P1 = ...  %p1 järjestettynä 3x4 matriisimuotoon

% Tarkistetaan että P1 projisoi pisteet oikein
% *************************************************************************
% * Kirjoita projektioyhtälö ja vertaa hiirella kliksuttelemiisi
% * koordinaatteihin.
% * MUISTA: Normalisoi homogeeninen koordinaatti jakolaskulle niin että w=1 
% * (ja x:hän oli x=[x y w]');
% *************************************************************************
%
% x1_proj = ... %Projektio
% x1_proj = x1_proj ... %Normalisointi
% x1 - x1_proj %Tästä pitäisi tulla jotakin pientä
% Mistä ero johtuu?

% *************************************************************************
% * Laske myös vähintään toiselle kameralle, aina hauskempaa kolmella tai
% * neljällä kameralla.
% *************************************************************************

% P2 = ...
% P3 = ...
% P4 = ...

%%
%--------------------------------------------------------------------------
%  Tehtävä 2)
%
%  Laske 3D piste X tunnetuista kameroista ja vastinpisteistä.
%
%  Nyt kun kaksi (tai useampi) kameraa (P1, P2, P3, P4) ovat tiedossa, 
%  voimme laskea uusia 3D pisteitä.
%
%  Jos meillä on pistekolmikko x1 <-> x2 <-> x3 (kaikki siis projektoita
%  samasta 3D pisteestä X) meillä on kolme yhtälöä
%  x1 = P1*X
%  x2 = P2*X
%  x3 = P3*X
% 
%  Tunnemme xi:t ja Pi:t. Käytetään samaa temppua kuin edellisessä
%  tehtävässä:
%  [x1]_x*x1 = [x1]_x*P1*X = 0
%  [x2]_x*x2 = [x2]_x*P2*X = 0
%  [x3]_x*x3 = [x3]_x*P3*X = 0
%
%  Jos tarkastellaan vain yhtälöä [x1]_x*P1*X = 0, voimme muuttaa tämän
%  muotoon A*X = 0. A (joka on 3x4 matriisi) on helppo laskea kun 
%  muistetaan että ristitulomatriisi on muotoa:
%  [a]_x = [ 0   -a_3  a_2;
%            a_3  0   -a_1;
%           -a_2  a_1  0  ];
%  
%  Voimme siis taas laskea jokaiselle havainnolle (tässä tapauksessa
%  kamera <-> projisoitunut piste) matriisin A ja kasata nämä päällekkäin
%  matriisiksi A_full. X saadaan samaan tapaan kuin edellisessä tehtävässä
%  p (SVD:llä).
%--------------------------------------------------------------------------

% *************************************************************************
% * Laske matriisit Ai tehtävänannon ohjeen mukaan (jokin matriisi kertaa
% * jokin toinen matriisi).
% *************************************************************************
% A1 = ...
% A2 = ...
% A3 = ...
% A_full = [A1;A2;A3];

% Nyt X on helppo ratkaista SVD:llä (X on matriisin A_full ydin)
% *************************************************************************
% * Laske X käyttäen SVD:tä kuten edellisen tehtävän 'p'
% * MUISTA: Normalisoi homogeeninen koordinaatti siten että W=1
% *************************************************************************
% [U S V] = svd(A_full, 0);
% X = ... %Oikeanpuoleinen singulaarivektori joka vastaa pienintä
%         %singulaariarvoa

% Toista muutamalle pisteelle ja plottaa pisteet 3D koordinaatistoon.
% Tämä on järkevintä tehdä 'for'-loopissa.

%%
%--------------------------------------------------------------------------
%  Tehtävä 3)
%
%  Muokkaa edellisen tehtävän scripi funktioksi joka ottaa parametreiksi
%  P_cell ja x_cell muuttujat ja palauttaa matriisin X.
%
%  function X = triangulate(P_cell, x_cell)
%  % Triangulate 3D point from known camera matrices and 2D points
%  %
%  % Inputs:
%  %       P_cell - Cell array of size 1xn (where n>=2). Each cell contains
%  %                  camera matrix P (size 3x4).
%  %       x_cell - Cell array of size 1xn. Each cell contains 3xm matrix
%  %                  of 2D homogenous points.
%  % Outputs:
%  %       X      - 4xm matrix of 3D homogenous points.
%  ...
%--------------------------------------------------------------------------

%%
%--------------------------------------------------------------------------
%  Tehtävä 4)
% 
%  Kohinan vaikutusta 3D pisteiden laskentaan.
%
%  Generoidaan suuri joukko 3D pisteitä ja luodaan pari kameraa käyttäen
%  apufunktiota 'getCamera'.
%
%  Luomme "realistisen" kalibrointimatriisin.
%
%  Lue ohjeet alta
%--------------------------------------------------------------------------

%Luodaan pisteet ja plotataan
[X_ Y_ Z_] = peaks();
X = [X_(:) Y_(:) Z_(:) ones(numel(X_),1)]';


figure(1)
plot3(X(1,:),X(2,:),X(3,:))

%Projektio plottaus päälle
set(gca,'projection','perspective')

disp('Liikuta plottia hiirellä (Rotate 3D) ja paina jotakin näppäintä jolloin nykyinen plotin kamera tallennetaan testikameraksi')
pause
%Tallennetaan kamera
[tmp R1 C1] = getCamera();
disp('Liikuta plottia uudelleen mielestäsi hyvään stereokamerakonfiguraatioon')
pause
[tmp R2 C2] = getCamera();
%% 
% Realistinen kalibrointimatriisi
K = [       2562.2    0.01       600.29
            0       2549.9       368.29
            0            0            1];

% Rakennetaan kamerat uudelleen
P1 = K*[R1 -R1*C1];
P2 = K*[R2 -R2*C2];

% Projisoidaan
x1 = P1*X;
x1 = bsxfun(@rdivide, x1, x1(end,:));

x2 = P2*X;
x2 = bsxfun(@rdivide, x2, x2(end,:));

% Ja plotataan
%figure(2)
subplot(1,2,1)
plot(x1(1,:),x1(2,:),'r.')
title('Ykköskamera')
subplot(1,2,2)
plot(x2(1,:),x2(2,:),'r.')
title('Kakkoskamera')
% HUOM: Data 2D tasossa

% Lasketaan 3D pisteet käyttäen triangulate funktiota joka tehtiin
% edellisessä tehtäväss
X_reconstruction = triangulate({P1, P2}, {x1,x2});
X_reconstruction = bsxfun(@rdivide, X_reconstruction, X_reconstruction(end,:));

%figure(3)
plot3(X_reconstruction(1,:),X_reconstruction(2,:),X_reconstruction(3,:))
title('3D data laskettuna projektioista (ei kohinaa)')
%% 
% Lisätään mittauspisteisiin hieman kohinaa ja lasketaan 3D pisteet
% uudelleen.
noise_amount_pix = 10; %***** Muuttele tätä arvoa ******
x1_noise = x1;
x1_noise(1:2,:) = x1_noise(1:2,:) + rand(size(x1_noise(1:2,:)))*noise_amount_pix;
x2_noise = x2;
x2_noise(1:2,:) = x2_noise(1:2,:) + rand(size(x1_noise(1:2,:)))*noise_amount_pix;

X_noise = triangulate({P1, P2}, {x1_noise,x2_noise});
X_noise = bsxfun(@rdivide, X_noise, X_noise(end,:));

figure(3)
plot3(X_noise(1,:),X_noise(2,:),X_noise(3,:))
title('3D data laskettuna projektioista (kohinaa)')

% Huomaa että 3D muoto säilyy tunnistettavana eikä "taivu"

%%
% Sen sijaan että kohina on vastinpisteissä, aiheutetaan kohinaa
% kalibrointimatriisiin:
noise_amount = 0.04; %***** Muuttele tätä arvoa ******
K_noise = K;
K_noise = K_noise + K_noise.*noise_amount.*(rand(size(K_noise))-0.5);

P1_noise = K_noise*[R1 -R1*C1];
P2_noise = K_noise*[R2 -R2*C2];

X_noise = triangulate({P1_noise, P2_noise}, {x1,x2});
X_noise = bsxfun(@rdivide, X_noise, X_noise(end,:));

figure(3)
plot3(X_noise(1,:),X_noise(2,:),X_noise(3,:))
title('3D data laskettuna projektioista (kohinaa)')

% Huomaa että 3D muoto repeilee ja taipuu (hieman riippuen kohinasta ja
% kameroiden sijainneista)

%%
% Kokeillaa vielä että jos kohina on kameran paikassa (ei rotaatiossa)
noise_amount = 0.14148; %***** Muuttele tätä arvoa ******
C1_noise = C1;
C1_noise = C1_noise + C1_noise.*noise_amount.*rand(size(C1_noise));
C2_noise = C2;
C2_noise = C2_noise + C2_noise.*noise_amount.*rand(size(C2_noise));


P1_noise = K*[R1 -R1*C1_noise];
P2_noise = K*[R2 -R2*C2_noise];

X_noise = triangulate({P1_noise, P2_noise}, {x1,x2});
X_noise = bsxfun(@rdivide, X_noise, X_noise(end,:));

figure(3)
plot3(X_noise(1,:),X_noise(2,:),X_noise(3,:))
title('3D data laskettuna projektioista (kohinaa)')

% Repeilee/aaltoilee herkästi varsinkin jos kameroiden pitäisi olla lähekkäin

% Mitä tästä opimme:
% Kameran kunnollinen kalibrointi ja paikan määrittäminen on 3D mittauksen
% kannalta vähintään yhtä tärkeää kuin vastinpisteiden löytäminen.