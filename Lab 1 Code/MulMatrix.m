function [Matrix] = MulMatrix(t,d,a,alph)
%D-H paramter as input, Transformation Matrix relating Frame i to i+1 is
%computed and returned
Matrix = [cos(t) -sin(t) 0 a; ...
        sin(t)*cos(alph) cos(t)*cos(alph) -sin(alph) -sin(alph)*d; ...
         sin(t)*sin(alph) cos(t)*sin(alph) cos(alph) cos(alph)*d; ...
        0 0 0 1];
end