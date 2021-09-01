%% volume computation
ptcd = pcread('a-cleaned.ply');%% preview
pcshow(ptcd)
% dt = delaunayTriangulation(double(ptcd.Location));
dt = delaunayTriangulation(double(ptcd.Location));
[ch, Volume] = convexHull(dt);
X = ptcd.Location(:,1);
Y = ptcd.Location(:,2);
Z = ptcd.Location(:,3);

points = [X(:),Y(:),Z(:)];

[normals,curvature] = findPointNormals(points,[],[0,0,10],true);


hold off;
surf(X,Y,Z,reshape(curvature,49,49));
hold on;
quiver3(points(:,1),points(:,2),points(:,3),...
    normals(:,1),normals(:,2),normals(:,3),'r');
axis equal;
    

    
    
    
    
    
    
normals = pcnormals(ptcd);

figure
pcshow(ptcd)
title('Estimated Normals of Point Cloud')
hold on

x = ptcd.Location(:,1);
y = ptcd.Location(:,2);
z = ptcd.Location(:,3);
u = normals(:,1);
v = normals(:,2);
w = normals(:,3);
quiver3(x,y,z,u,v,w);
hold off


sensorCenter = [0,-0.3,0.3]; 
for k = 1 : numel(x)
   p1 = sensorCenter - [x(k),y(k),z(k)];
   p2 = [u(k),v(k),w(k)];
   % Flip the normal vector if it is not pointing towards the sensor.
   angle = atan2(norm(cross(p1,p2)),p1*p2');
   if angle > pi/2 || angle < -pi/2
       u(k) = -u(k);
       v(k) = -v(k);
       w(k) = -w(k);
   end
end


figure
pcshow(ptcd)
title('Adjusted Normals of Point Cloud')
hold on
quiver3(x, y, z, u, v, w);
hold off




shp = alphaShape(x,y,z)
