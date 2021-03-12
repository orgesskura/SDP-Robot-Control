
# high parameters
IGNORE_GAP = 10

# get direction which boat should ahead
def left_forward_right(gap):
    if gap < 0 and abs(gap) > IGNORE_GAP:
        return "turn_left"
    elif gap > 0 and abs(gap) > IGNORE_GAP:
        return "turn_right"
    else:
        return "go_forward"