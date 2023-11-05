function PickAndSort(r,r2,vert,item,n,qItem,vertItem,item2)

    qD0 = [0    0.7854    1.5708    0.7854         0];
    qD1 = [-1.5708    0.0873    0.5345    2.4871         0];
    qD2 = qItem;
    qD3 = [1.5708    0.0873    0.5345    2.4871         0];
    qD4 = [1.5708    0.8781    0.1418    2.0944    0];

    switch n
        case 1
            BoxFunc1(r,vert,item);
        case 2
            BoxFunc2(r,vert,item);
        case 3
            BoxFunc3(r,vert,item);
    end

    ItemFuncMove(r2, qD0, qD1, 100, vert, item,0);

    ItemFuncMove(r2, qD1, qD2, 100, vert, item,0);

    EndEffItem1 = r2.model.fkine(r2.model.getpos()).T;
    updatedVertItem1 = (EndEffItem1*[vertItem,ones(size(vertItem,1),1)]')';
    item2.Vertices = updatedVertItem1(:,1:3);
    drawnow();

    ItemFuncMove(r2, qD2, qD3, 100, vert, item,2, vertItem, item2);

    ItemFuncMove(r2, qD3, qD4, 100, vert, item,2, vertItem, item2);

    EndEffItem1 = r.model.fkine(r.model.getpos()).T * transl(0.15,0.1,0.1);
    updatedVertItem1 = (EndEffItem1*[vertItem,ones(size(vertItem,1),1)]')';
    item2.Vertices = updatedVertItem1(:,1:3);
    drawnow();

    ItemFuncMove(r2, qD4, qD3, 100, vert, item,0, vertItem, item2);

    ItemFuncMove(r2, qD3, qD0, 200, vert, item,0, vertItem, item2);

    switch n
        case 1
            BoxBack1(r,vert,item,2, vertItem, item2);
        case 2
            BoxBack2(r,vert,item,2, vertItem, item2);
        case 3
            BoxBack3(r,vert,item,2, vertItem, item2);
    end
    
    pause(3)
end