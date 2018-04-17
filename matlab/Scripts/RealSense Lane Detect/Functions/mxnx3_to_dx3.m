function dx3 = mxnx3_to_dx3(mxnx3)
[height, width, length] = size(mxnx3);
dx3 = zeros(height*width,3);

for i = 1:length
    for j = 1:height
        for k = 1:width
            l = width*(j-1)+k;
            dx3(l,i) = mxnx3(j,k,i);
        end
    end
end

end