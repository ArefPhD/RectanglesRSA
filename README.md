# RectanglesRSA

%generation of polygons here polygons with 10 sides tracking successful and total attempts at placing polygons 

clear;close all;
L=[1];
a=0.5;
tic
sz=1;
range=50;
Side=10;%square =4, pentagon=5
[inradius, circumradius]=polygon_radii(Side, a);
rsmall=2*inradius-0.2;
rBig=2*circumradius+0.2;

for i=1:1
    V(i,:)=[i,range*rand,range*rand,1,1]
end

n=1;

LP=size(L(:,1));

nn=LP(1,1)+1;
KLOP=100000;
T_itr=0;
S_itr=1;
while nn<KLOP
    T_itr=T_itr+1;
    
    A=range*rand;
    B=range*rand;
    newmat=(V(:,2)<(A+rBig) & V(:,2)>(A-rBig)& V(:,3)<(B+rBig) & V(:,3)>(B-rBig));
    n2=1;
    SP=[0 0 0 0 0];
    for i=1:length(newmat)
        if newmat(i,1)==1
        SP(n2,:)=V(i,:);
        n2=n2+1;
        end
    end
    
    for i=1:length(SP(:,1))
    pgon1 = nsidedpoly(Side,'Center',[SP(i,2) SP(i,3)],'SideLength',a);

    pgon2 = nsidedpoly(Side,'Center',[A B],'SideLength',a);

polyout = intersect(pgon1,pgon2);


    D(i,1) =polyout.NumRegions;
    
    
    
   
    end
    % left border checking whether it has crossed.
        pgon1=nsidedpoly(4,'Center',[-range/2 range/2],'SideLength',range);
        polyout = intersect(pgon1,pgon2);
        D(i+1,1)=polyout.NumRegions;
        
        pgon1=nsidedpoly(4,'Center',[3/2*range range/2],'SideLength',range);
        polyout = intersect(pgon1,pgon2);
        D(i+2,1)=polyout.NumRegions; 
        
        pgon1=nsidedpoly(4,'Center',[range/2 -range/2],'SideLength',range);
        polyout = intersect(pgon1,pgon2);
        D(i+3,1)=polyout.NumRegions; 
        
        pgon1=nsidedpoly(4,'Center',[range/2 3/2*range],'SideLength',range);
        polyout = intersect(pgon1,pgon2);
        D(i+4,1)=polyout.NumRegions; 
        
    S=D(:);
   
     if sum(S(:))==0
        K=length(V(:,1));
        V(K+1,2)=A;V(K+1,3)=B;
        %ppp(K+1,1)=nsidedpoly(Side,'Center',[A B],'SideLength',a);
             L(nn,1)=S_itr;
        L(nn,2)=K;
        L(nn,3)=T_itr;
        nn=nn+1;
        S_itr=S_itr+1;
     end
     %LS=[distance;A;B;C;nn]
D=[0]; 

         size(V)
         save('square_smallarea.m')

end

function [inradius, circumradius] = polygon_radii(n, s)
    % Calculate circumradius using side length
    circumradius = s / (2 * sin(pi/n));
    
    % Calculate inradius using circumradius and side length
    inradius = circumradius * cos(pi/n);
end

%showing polygons
hold
  for i=1:length(V(:,1))
   pgon1=nsidedpoly(Side,'Center',[V(i,2),V(i,3)],'SideLength',a);

    plot(pgon1 ,  'HoleEdgeColor',[0 0 0],...
    'EdgeColor',[0 0 0],...
    'FaceColor',[0 0 1],...
    'FaceAlpha',0.85);
  end
