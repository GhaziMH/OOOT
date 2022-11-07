clear all
close all
clc

Files = dir(fullfile('C:', 'Users', 'galon','source', 'repos','OOOT','Data','*.CSV'));

Theta=[] ;
Phi=[] ;

for i=1:1:21
    Theta(i) = (i-1)*pi/(21-1);
end
for i=1:1:41
    Phi(i) = (i-1)*pi/(21-1);
end
Title = "";
counter = 0;
for i=1:size(Files)
    counter = counter + 1;
    if(counter == 2)
                Title = 'Orientation Evaluation X1';
    elseif (counter == 3)
                    Title = 'Orientation Evaluation X2';
    elseif (counter == 4)
        Title = 'Orientation Evaluation X3';
    elseif(counter == 5 || counter ==1)
        counter = 1;
        disp('next object');
        Title = 'Orientation Evaluation Weighted Average ';
    end
    figure(i);
    data = readtable(Files(i).name);
    data = data{:,:};
    %h= heatmap(Phi,Theta, data, 'GridVisible', 'off','ColorLimits',[0,1], Colormap= jet);
    h= heatmap(Phi,Theta, data, 'GridVisible', 'off', Colormap= jet);
    %%set(h,'XTick',0:pi/40:pi) 
    %%set(h,'XTickLabel',0:pi/40:pi)
    %%set(h,'YTick',0:pi/20:pi) 
    %%set(h,'YTickLabel',0:pi/20:pi)
    h.Title = Title;
    h.XLabel = 'Phi';
    h.YLabel = 'Theta';

end




