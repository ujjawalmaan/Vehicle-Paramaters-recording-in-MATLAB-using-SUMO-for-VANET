function saveimage_publish(imagename)
fig = gcf;
fig.PaperUnits = 'inches';
% fig.PaperPosition = [0 0 7 3];
print(strcat(imagename),'-dpng','-r0')