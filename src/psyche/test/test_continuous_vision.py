import pytest
from psyche.continuous_vision import ContinuousVision

def test_generate_prompt():
    continuous_vision = ContinuousVision()
    prompt_template = "Enter your prompt: {prompt}"
    inputs = {
        'prompt': 'Hello, world!',
        'images': ['image1', 'to_excise', 'to_excise', 'image2', 'to_excise', 'to_excise', 'image3']
    }
    expected_response = {
        'prompt': 'Enter your prompt: Hello, world!',
        'images': ['image1', 'image2', 'image3']
    }
    response = continuous_vision.generate_prompt(prompt_template, inputs)
    assert response == expected_response