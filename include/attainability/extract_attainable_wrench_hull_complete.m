function [cell_bndry_af, cell_bndry_am] = extract_attainable_wrench_hull_complete(bndry_af, bndry_am)
    N_nodes = size(bndry_af, 1);
    cell_bndry_af = cell(1, N_nodes);
    cell_bndry_am = cell(1, N_nodes);
    
    for i_node = 1 : N_nodes
        i_bndry_inode = boundary(bndry_af(i_node, :)', bndry_am(i_node, :)', 0);

        cell_bndry_af{i_node} = bndry_af(i_node, i_bndry_inode);
        cell_bndry_am{i_node} = bndry_am(i_node, i_bndry_inode);
    end
end

