def is_prime_number(n):
    assert isinstance(n, int), "Did you cast to int in Q1.2?"
    """Return whether given integer is a prime_number.

    >>> is_prime_number(9)
    False
    >>> is_prime_number(2011)
    True

    """
    # BEGIN QUESTION 1.1
    "*** REPLACE THIS LINE ***"
    x=0
    y=str(n)
    if n==2 or n==3:
        return True
    elif n>3:
        for i in y:
            x=x+int(y)
        if n%6==1 or (n%2!=0 and x%3!=0):
            return True
        else:
            return False
    else:
        return False
    # END QUESTION 1.1
