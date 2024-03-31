def require_args(required_args: list[str]):
    """
    Takes a list of required arguments and checks if they are present in every function call. If any of the required arguments are missing, a TypeError is raised. 
    @param required_args - a list of required arguments
    @return function decorated with argument checker.
    """
    def decorator(func):
        def wrapper(*args, **kwargs):
            for name in required_args:
                if name not in kwargs:
                    raise TypeError(
                        f"Required argument '{name}' not provided in {func.__name__}.")
            return func(*args, **kwargs)
        return wrapper
    return decorator

class ResultTag:
    DEAD_END: str = 'Dead-end'
    LOOP: str = 'Stuck in a loop'
    LOCAL_MINIMUM: str = 'Local minimum'
    SUCCESS: str = 'Destination was reached'
    FACE: str = 'Face was traversed'
    NO_PROGRESS: str = 'Current closest node is the same as in the previous iteration'
