%get the data
points = readtable('D:\western uni\9047\lab3\Graph_251462513.txt');

%plot the nodes and blue circles
plot(points.Var1,points.Var2,'bo');

%keep the plot active
hold on;

%this is the number of nodes
N = 20;
%this is the communications radius
RC = 0.25;

%loop through all nodes
for p =1:N
    %loop through rest of nodes again
    for q=p+1:N
        %calculate the distance between two nodes
        d = sqrt((points.Var1(p)-points.Var1(q))^2 + sqrt(points.Var2(p)-points.Var2(q))^2)
        %if distance is less than communications radius, draw a line
        if(d<RC)
            plot ([ points.Var1(p), points.Var1(q)],[ points.Var2(p), points.Var2(q)], 'r-');
        end
    end
end
