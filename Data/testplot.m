clear all 
clc

figure;
model = stl2matlab('TestPartC.STL');

patch(model{3},model{2},model{1},'k',EdgeColor='red');
axis equal; view(30,60);
hold on 
alpha (1);
v1 = min(min(model{3}))-10;
v2 = min(min(model{2}))-10;
v3 = min(min(model{1}))+30;
v4 = max(max(model{3}))+10;
v5 = max(max(model{2}))+10;
v6 = max(max(model{1}))-10;


xd=linspace(min(min(model{3}))-10,max(max(model{3}))+10,10);
yd=linspace(min(min(model{2}))-10,max(max(model{2}))+10,10);
[x,y]=meshgrid(xd,yd);

z=-x*0 + min(min(model{1}))+30;

% first plane

mesh(x,y,z,"FaceColor","b",EdgeColor="none");
alpha(0.8);
%patch([v1,v1,v4,v4,v1],[v2,v5,v5,v2,v2],[v6,v6,v6,v6,v6],'g')


function [VertexData,FVCD,isBinary]=stl2matlab(stlfile)
% STL2MATLAB reads STL-file, ASCII or binary format, into Matlab
%
% Usage:
%
% [VertexData, FVCD]=stl2matlab(stlfile)
%
% Input:
%
% stlfile - STL-file
%
% Output:
%
% VertexData - Cellarray containing matrices with vertices
% FVCD - FaceVertexColorData
% isBinary - Flag with STL-file information
%
% To plot the STL-object use plotSTL(VertexData) or plotSTL(VertexData,FVCD)
%
% m-file can be downloaded at
% http://www.mathworks.com/matlabcentral/fileexchange/13253-iges-toolbox
%
% written by Per Bergstrm 2012-01-09
%

if nargin == 0
    warning('No STL-file is specified');
end

VertexData=cell(1,3);

try
    
    % Try to read an STL ASCII file
    [VertexData{1}, VertexData{2}, VertexData{3}, FVCD] = stlAread(stlfile);
    isBinary=false;
    
catch
    
    try
        
        % Try to read an STL binary file
        [VertexData{1}, VertexData{2}, VertexData{3}, FVCD] = stlBread(stlfile);
        isBinary=true;
        
    catch
        
        error('File could not be read!')
        
    end
    
end


end


function [X, Y, Z, FVCD] = stlAread(stlfile)
% Reads an STL ASCII file

fid=fopen(stlfile,'r');

fileTitle=sscanf(fgetl(fid),'%*s %s');

vnum=0;
fclr=0;
testASCII=true;
lineCount=0;

while feof(fid) == 0
    stlLine=fgetl(fid);
    keyWord=sscanf(stlLine,'%s');
    if strncmpi(keyWord,'c',1) == 1
        fclr=sscanf(stlLine,'%*s %f %f %f');
    elseif strncmpi(keyWord,'v',1) == 1
        vnum=vnum+1;
        vertex(:,vnum)=sscanf(stlLine,'%*s %f %f %f');
        clr(:,vnum)=fclr;
    elseif testASCII
        lineCount=lineCount+1;
        if lineCount>20
            if vnum>2
                testASCII=false;
            else
                error('File is not an STL ASCII file!')
            end
        end
    end
end

X=[vertex(1,1:3:end);vertex(1,2:3:end);vertex(1,3:3:end)];
Y=[vertex(2,1:3:end);vertex(2,2:3:end);vertex(2,3:3:end)];
Z=[vertex(3,1:3:end);vertex(3,2:3:end);vertex(3,3:3:end)];

FVCD=reshape(clr,3,vnum/3);

fclose(fid);

end
function [X, Y, Z, FVCD] = stlBread(stlfile)
% Reads an STL binary file

fid=fopen(stlfile,'r');

fileTitle=fread(fid,80,'uchar=>schar');
fnum=fread(fid,1,'int32');

X=zeros(3,fnum);
Y=zeros(3,fnum);
Z=zeros(3,fnum);

FVCD=uint8(zeros(3,fnum));

for i=1:fnum
    normal=fread(fid,3,'float32');
    vertex1=fread(fid,3,'float32');
    vertex2=fread(fid,3,'float32');
    vertex3=fread(fid,3,'float32');
    clr=fread(fid,1,'uint16');
    if bitget(clr,16)==1
        rd=bitshift(bitand(65535,clr),-10);
        grn=bitshift(bitand(2047,clr),-5);
        bl=bitand(63,clr);
        FVCD(:,i)=[rd;grn;bl];
    end
    X(:,i)=[vertex1(1); vertex2(1); vertex3(1)];
    Y(:,i)=[vertex1(2); vertex2(2); vertex3(2)];
    Z(:,i)=[vertex1(3); vertex2(3); vertex3(3)];
end

fclose(fid);
end