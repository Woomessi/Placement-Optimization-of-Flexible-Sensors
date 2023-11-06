clear
clc

item = 0:0.1:1;
size_group = 55;
group = zeros(55,3);
round = 1;
for i = 1:11
    barycentric1 = item(i);
    for j = 1:11
        barycentric2 = item(j);
        if barycentric1 + barycentric2 >1
            break
        end
        group(round,:) = [barycentric1,barycentric2,1 - barycentric1 - barycentric2];
        round = round + 1;
    end
end