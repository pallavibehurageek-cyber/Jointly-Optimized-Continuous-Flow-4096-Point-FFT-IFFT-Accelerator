function save_complex_vector(filename,x)

fid = fopen(filename,'w');

for k=1:length(x)

    fprintf(fid,"%d %d\n",...
        round(real(x(k))),...
        round(imag(x(k))));

end

fclose(fid);

end
