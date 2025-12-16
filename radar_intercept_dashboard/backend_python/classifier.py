# classifier.py - classify by cross_section area
def classify_by_cross_section(area):
    if area < 1.5:
        return 'drone'
    if area < 10.0:
        return 'missile'
    return 'plane'
