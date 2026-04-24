function arr = insertAtIndex(arr, index, value)
low = arr(1 : index - 1);
high = arr(index : end);
arr = [low; value; high];
end

