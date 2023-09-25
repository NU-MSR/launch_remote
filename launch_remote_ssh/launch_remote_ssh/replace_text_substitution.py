from typing import Text
from typing import Iterable

from launch.substitution import Substitution
from launch.launch_context import LaunchContext
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

class ReplaceTextSubstitution(Substitution):
    """Substitution that replaces text1 with text2 in an input substitution."""
    def __init__(self, substitutions: Iterable[Substitution], text1: Text, text2: Text) -> None:
        """Create a ReplaceTextSubstitution."""
        super().__init__()

        if not isinstance(text1, Text):
            raise TypeError(
                "ReplaceTextSubstitution expected Text object got '{}' instead.".format(type(text1))
            )
        if not isinstance(text2, Text):
            raise TypeError(
                "ReplaceTextSubstitution expected Text object got '{}' instead.".format(type(text2))
            )
        self.__substitutions = normalize_to_list_of_substitutions(substitutions)
        self.__text1 = text1
        self.__text2 = text2

    @property
    def substitutions(self) -> Substitution:
        """Getter for text1."""
        return self.__substitutions

    @property
    def text1(self) -> Text:
        """Getter for text1."""
        return self.__text1
    
    @property
    def text2(self) -> Text:
        """Getter for text1."""
        return self.__text2
    
    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return f"Replace '{self.text1}' with '{self.text2}."

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by returning the string itself."""

        performed_substitution = perform_substitutions(context, self.__substitutions)
        return performed_substitution.replace(self.text1, self.text2)