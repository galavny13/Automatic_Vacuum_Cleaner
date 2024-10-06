#ifndef BUTTON_H
#define BUTTON_H

#if USE_GUI
#include <SFML/Graphics.hpp>
#include <functional>
#endif

class Button {
public:

#if USE_GUI
    /**
     * Constructs a Button object.
     * @param position The position of the button.
     * @param size The size of the button.
     * @param text The initial text of the button.
     * @param font The font to use for the button text.
     * @param characterSize The size of the text characters.
     */
    Button(const sf::Vector2f& position, const sf::Vector2f& size, const std::string& text, const sf::Font& font, unsigned int characterSize = 16)
            : m_shape(size), m_text(text, font, characterSize) {
        m_shape.setPosition(position);
        m_shape.setFillColor(sf::Color(200, 200, 200));
        m_shape.setOutlineThickness(2);
        m_shape.setOutlineColor(sf::Color::Black);

        m_text.setFillColor(sf::Color::Black);
        centerText();
    }

    /**
     * Draws the button on the given window.
     * @param window The render window to draw on.
     */
    void draw(sf::RenderWindow& window) const {
        window.draw(m_shape);
        window.draw(m_text);
    }

    /**
     * Checks if the mouse is over the button.
     * @param mousePos The current mouse position.
     * @return True if the mouse is over the button, false otherwise.
     */
    bool isMouseOver(const sf::Vector2f& mousePos) const {
        return m_shape.getGlobalBounds().contains(mousePos);
    }

    /**
     * Sets the callback function for when the button is clicked.
     * @param callback The function to call when the button is clicked.
     */
    void setOnClick(const std::function<void()>& callback) {
        m_onClick = callback;
    }

    /**
     * Handles a click on the button.
     */
    void handleClick() {
        if (m_onClick) {
            m_onClick();
        }
    }

    /**
     * Sets the text of the button.
     * @param text The new text for the button.
     */
    void setText(const std::string& text) {
        m_text.setString(text);
        centerText();
    }


    void setEnabled(bool enabled) {
        m_enabled = enabled;
        updateColor();
    }

    bool isEnabled() const {
        return m_enabled;
    }
#endif

private:

#if USE_GUI
    sf::RectangleShape m_shape;
    sf::Text m_text;
    std::function<void()> m_onClick;
    bool m_enabled = true;


    /**
     * Centers the text within the button.
     */
    void centerText() {
        sf::FloatRect textBounds = m_text.getLocalBounds();
        m_text.setOrigin(textBounds.left + textBounds.width / 2.0f, textBounds.top + textBounds.height / 2.0f);
        m_text.setPosition(
                m_shape.getPosition().x + m_shape.getSize().x / 2.0f,
                m_shape.getPosition().y + m_shape.getSize().y / 2.0f + 2.0f  // Add a small offset to move text down
        );
    }


    void updateColor() {
        if (m_enabled) {
            m_shape.setFillColor(sf::Color(200, 200, 200));  // Light gray for enabled buttons
        } else {
            m_shape.setFillColor(sf::Color(255, 99, 71));  // Tomato red for disabled buttons
        }
    }
#endif
};

#endif // BUTTON_H