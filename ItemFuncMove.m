function ItemFuncMove(r, Q1, Q2, n, vert, Item,m, vertItem, Item2)


traj = jtraj(Q1,Q2,n);

    for i = 1:n
        pause(0.02);
        r.model.animate(traj(i, :));      
        switch m
            case 0
            case 1
                EndEff = r.model.fkine(r.model.getpos()).T;
                updatedVert = (EndEff*[vert,ones(size(vert,1),1)]')';
                Item.Vertices = updatedVert(:,1:3);
                drawnow();
            case 2
                EndEffItem1 = r.model.fkine(r.model.getpos()).T ;
                updatedVertItem1 = (EndEffItem1*[vertItem,ones(size(vertItem,1),1)]')';
                Item2.Vertices = updatedVertItem1(:,1:3);
                drawnow();
            case 3
                EndEff = r.model.fkine(r.model.getpos()).T;
                updatedVert = (EndEff*[vert,ones(size(vert,1),1)]')';
                Item.Vertices = updatedVert(:,1:3);
                drawnow();
                EndEffItem1 = r.model.fkine(r.model.getpos()).T * transl(0,0,0.1);
                updatedVertItem1 = (EndEffItem1*[vertItem,ones(size(vertItem,1),1)]')';
                Item2.Vertices = updatedVertItem1(:,1:3);
                drawnow();

        end
            
    end

end

