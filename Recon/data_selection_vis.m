i = 1;
dire = uigetdir;
files = 0;
while i>0
    
    try
        imds = imageDatastore(dire); 
        file = length(imds.Files);
        
        if file >= 12
            break
        end
    catch
       i= i+1;
    end
    drawnow
 
end
   