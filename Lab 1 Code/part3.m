clear;

%D-H Paramters
%Distance from Zi to Zi+1 along Xi
a = [0 0 sym('L2','real') 0 0 0];
%a = [0 0 172.6 0 0 0];
%Angle between Zi to Zi+1 along Xi
alph = [0 pi/2 0 pi/2 -pi/2 pi/2];
%Angle between Xi to Xi+1 along Zi
t = [sym('T1','real') sym('T2','real') sym('T3','real') sym('T4','real') sym('T5','real') sym('T6','real')];
dt = [sym('DT1','real') sym('DT2','real') sym('DT3','real') sym('DT4','real') sym('DT5','real') sym('DT6','real')];
%Distance from Xi to Xi+1 along Zi
d = [sym('L1','real') 0 0 sym('L3','real') 0 sym('L4','real')];
%d = [117.1 0 0 108.7 0 74.4];

% %D-H Paramters
% %Distance from Zi to Zi+1 along Xi
% a = [0 0 172.6 0 0 0];
% %Angle between Zi to Zi+1 along Xi
% alph = [0 90 0 90 -90 90];
% %Angle between Xi to Xi+1 along Zi
% t = [0 90 90 0 0 0];
% dt = [sym('DT1') sym('DT2') sym('DT3') sym('DT4') sym('DT5') sym('DT6')];
% %Distance from Xi to Xi+1 along Zi
% d = [117.1 0 0 108.7 0 74.4];
% % Desired Position
% Px = 0.3;
% Py = 0.15;
% Pz = 0.25;

%For loop to iterate through all frames
for n = 1:length(a)
    %Calling function to compute frame i to frame i+1 matrix
    Temp = MulMatrix(t(n),d(n),a(n),alph(n));
    %If its the first frame, store the result so it doesn't get multiplied
    %by zero, otherwise multiply it by the previous value
    if (n == 1)
        M = Temp;
        m1 =[Temp(1) Temp(5) Temp(9); Temp(2) Temp(6) Temp(10); Temp(3) Temp(7) Temp(11)];
        P1 = [Temp(13); Temp(14); Temp(15)];
    else 
        M = M * Temp;
        if (n==2)
            %If it is the second frame, store the the indiviudal frame 
            %rotational matrix in m2 and the the translation in P2
            %for later calculations
            m2 =[Temp(1) Temp(5) Temp(9); Temp(2) Temp(6) Temp(10); Temp(3) Temp(7) Temp(11)];
       P2 = [Temp(13); Temp(14); Temp(15)];
        end
        if (n==3)
            %If it is the third frame, store the the indiviudal frame 
            %rotational matrix in m3 and the the translation in P3
            %for later calculations
            m3 =[Temp(1) Temp(5) Temp(9); Temp(2) Temp(6) Temp(10); Temp(3) Temp(7) Temp(11)];
        P3 = [Temp(13); Temp(14); Temp(15)];
        end
        if (n==4)
            %If it is the second frame, store the the indiviudal frame 
            %rotational matrix in m2 and the the translation in P2
            %for later calculations
            m4 =[Temp(1) Temp(5) Temp(9); Temp(2) Temp(6) Temp(10); Temp(3) Temp(7) Temp(11)];
       P4 = [Temp(13); Temp(14); Temp(15)];
        end
        if (n==5)
            %If it is the third frame, store the the indiviudal frame 
            %rotational matrix in m3 and the the translation in P3
            %for later calculations
            m5 =[Temp(1) Temp(5) Temp(9); Temp(2) Temp(6) Temp(10); Temp(3) Temp(7) Temp(11)];
        P5 = [Temp(13); Temp(14); Temp(15)];
        end
        if (n==6)
            %If it is the third frame, store the the indiviudal frame 
            %rotational matrix in m3 and the the translation in P3
            %for later calculations
            m6 =[Temp(1) Temp(5) Temp(9); Temp(2) Temp(6) Temp(10); Temp(3) Temp(7) Temp(11)];
        P6 = [Temp(13); Temp(14); Temp(15)];
        end
    end
end
    
    for i=1:3
        for j=1:6
            
         Jacob1(i,j) = diff(M(i,4),t(j));
        end
    end
    
    aVelocity1 = [0;0;dt(1)];
lVelocity1 = [0; 0; 0];

aVelocity2 = simplify((m2.')*aVelocity1 + [0;0;dt(2)]);
lVelocity2 = simplify((m2.')*(lVelocity1 + cross(aVelocity1,P2)));

aVelocity3 = simplify((m3.')*aVelocity2 + [0;0;dt(3)]);
lVelocity3 = simplify((m3.')*(lVelocity2 + cross(aVelocity2,P3)));

aVelocity4 = simplify((m4.')*aVelocity3 + [0;0;dt(4)]);
lVelocity4 = simplify((m4.')*(lVelocity3 + cross(aVelocity3,P4)));

aVelocity5 = simplify((m5.')*aVelocity4 + [0;0;dt(5)]);
lVelocity5 = simplify((m5.')*(lVelocity4 + cross(aVelocity4,P5)));

aVelocity6 = simplify((m6.')*aVelocity5 + [0;0;dt(6)]);
lVelocity6 = simplify((m6.')*(lVelocity5 + cross(aVelocity5,P6)));

FinalM = simplify(m1*m2*m3*m4*m5*m6);
AngVel = simplify(FinalM * aVelocity6);
    for i=1:3
        for j=1:6
            
         Jacob2(i,j) = simplify(diff(AngVel(i),dt(j)));
        end
    end
Jacobian  = [Jacob1(1) Jacob1(4) Jacob1(7) Jacob1(10) Jacob1(13) Jacob1(16); Jacob1(2) Jacob1(5) Jacob1(8) Jacob1(11) Jacob1(14) Jacob1(17); Jacob1(3) Jacob1(6) Jacob1(9) Jacob1(12) Jacob1(15) Jacob1(18); Jacob2(1) Jacob2(4) Jacob2(7) Jacob2(10) Jacob2(13) Jacob2(16); Jacob2(2) Jacob2(5) Jacob2(8) Jacob2(11) Jacob2(14) Jacob2(17); Jacob2(3) Jacob2(6) Jacob2(9) Jacob2(12) Jacob2(15) Jacob2(18)];
%test1 = subs(Jacobian, [sym('T1','real') sym('T2','real') sym('T3','real') sym('T4','real') sym('T5','real') sym('T6','real')], [1, 2,3,4,5,6]);